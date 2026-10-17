# Kraken X60 connector cap built from the LIBRARY motor itself (owner: "align it properly and not off-angle. the flat
# side should fit with the kraken"). In library space the motor's back end has an open step: a floor ~11.4 mm below the
# back face and a flat wall. The cap is the motor's own outline just below that floor, cut exactly at the flat wall and
# extruded from the floor to the back face, so its sides continue the motor's sides, its top is flush with the back face
# and its flat side lies on the wall. Its bottom and flat-side faces would only sit on the motor's own floor and wall
# (never visible, coplanar back to back), so they are left out. Writes assets/kraken_cap_x60.json (library space as
# pipeline.lib_mesh builds it: the asset mesh through its object's world matrix).
#   blender -b --factory-startup --python kraken_cap_asset.py -- [out.json]
import bpy, sys, os, json
import numpy as np
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geo, build, align

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT = argv[0] if argv else os.path.join(HERE, "assets", "kraken_cap_x60.json")
KR = ("WCP-0940 (Kraken X60)/WCP-0940 (Kraken X60).blend", "WCP-0940 (Kraken X60)")
MM = 1e-3

sc = bpy.context.scene
ob = align.append_object(f"{align.LIB}/{KR[0]}", KR[1])
sc.collection.objects.link(ob)
d2 = bpy.context.evaluated_depsgraph_get()
me = bpy.data.meshes.new_from_object(ob.evaluated_get(d2), preserve_all_data_layers=True, depsgraph=d2)
me.transform(ob.matrix_world)
bpy.data.objects.remove(ob)

co = np.array([v.co[:] for v in me.vertices])
d = np.array((0.0, 0.0, 1.0))  # motor axis; the back face (with the notch) is at +Z
zmax = float(co[:, 2].max())
# notch floor: the largest +Z face area below the back face (more than 1 mm down)
floor = {}
for p in me.polygons:
    if p.normal.z > 0.999 and p.center.z < zmax - MM:
        k = round(p.center.z, 5)
        floor[k] = floor.get(k, 0.0) + p.area
zf = max(floor, key=floor.get)
# flat wall: the largest face perpendicular to the axis spanning floor -> back face; its normal points into the notch
wall = max((p for p in me.polygons if abs(p.normal.z) < 1e-3
            and min(me.vertices[i].co.z for i in p.vertices) < zf + 0.1 * MM
            and max(me.vertices[i].co.z for i in p.vertices) > zmax - 0.1 * MM), key=lambda p: p.area)
wn = np.array(wall.normal[:])
wp = np.array(wall.center[:])
print(f"back face z {zmax / MM:.3f} mm, notch floor z {zf / MM:.3f} mm ({floor[zf] / MM**2:.0f} mm2), "
      f"wall normal {np.round(wn, 5)}, wall at {np.round(wp / MM, 3)} mm ({wall.area / MM**2:.0f} mm2)")

# the motor's outline just below the floor; the far side of the wall above the floor must continue it (taper only)
loops, u, w = build.section_loops(me, d, zf - 0.01 * MM)
outline = max((np.asarray(L, float) for L in loops), key=lambda L: abs(geo.poly_area(L)))
n2 = np.array([wn @ u, wn @ w])
c2 = np.array([wp @ u, wp @ w])
sd = lambda q: (q - c2) @ n2
top = max((np.asarray(L, float) for L in build.section_loops(me, d, zmax - 0.01 * MM)[0]), key=lambda L: abs(geo.poly_area(L)))
seg_dist = lambda q, P: min(np.linalg.norm(q - (P[i] + np.clip((q - P[i]) @ (P[(i + 1) % len(P)] - P[i]) / max((P[(i + 1) % len(P)] - P[i]) @ (P[(i + 1) % len(P)] - P[i]), 1e-18), 0, 1) * (P[(i + 1) % len(P)] - P[i]))) for i in range(len(P)))
taper = max(seg_dist(q, outline) for q in top if sd(q) < -0.5 * MM)
print(f"outline at the floor: {len(outline)} pts; the far side at the back face is within {taper / MM:.3f} mm of it")
# keep the notch side of the wall plane (Sutherland-Hodgman, one half-plane)
clip = []
for i in range(len(outline)):
    a, b = outline[i], outline[(i + 1) % len(outline)]
    sa, sb = sd(a), sd(b)
    if sa >= -1e-7:
        clip.append(a)
    if (sa >= -1e-7) != (sb >= -1e-7):
        clip.append(a + (b - a) * (sa / (sa - sb)))
keep = []
for q in clip:  # drop near-duplicates (a section vertex lying on the wall)
    if not keep or np.linalg.norm(q - keep[-1]) > 0.01 * MM:
        keep.append(q)
if len(keep) > 1 and np.linalg.norm(keep[-1] - keep[0]) <= 0.01 * MM:
    keep.pop()
clip = np.array(keep)
cross2 = lambda a, b: a[0] * b[1] - a[1] * b[0]
straight = lambda i: abs(cross2(clip[i] - clip[i - 1], clip[(i + 1) % len(clip)] - clip[i])) < 1e-4 * np.linalg.norm(
    clip[i] - clip[i - 1]) * np.linalg.norm(clip[(i + 1) % len(clip)] - clip[i])
clip = clip[[i for i in range(len(clip)) if not straight(i)]]  # a section vertex in the middle of a straight side
if geo.poly_area(clip) < 0:  # counter-clockwise in (u, w) -> normal along +d (u x w = d)
    clip = clip[::-1]
cap = build.build_prism("kraken cap", clip, [], u, w, d, zf, zmax)
build.fix_normals(cap)
dn = Vector(d)
wnv = Vector(wn)
drop = [p.index for p in cap.polygons if p.normal.dot(dn) < -0.999 or p.normal.dot(wnv) < -0.999]  # bottom, flat side
import bmesh
bm = bmesh.new(); bm.from_mesh(cap)
bm.faces.ensure_lookup_table()
bmesh.ops.delete(bm, geom=[bm.faces[i] for i in drop], context='FACES_ONLY')
bm.to_mesh(cap); bm.free()
verts = [list(v.co[:]) for v in cap.vertices]
faces = [list(p.vertices) for p in cap.polygons]
json.dump({"asset": KR[1], "space": "library space as pipeline.lib_mesh builds it (asset mesh through its object's world matrix, metres)",
           "source": "kraken_cap_asset.py: the library Kraken X60's own outline at its rear step floor, cut at the step's flat "
                     "wall, extruded floor -> back face; bottom and flat-side faces left out (they lie on the motor's own faces)",
           "verts": verts, "faces": faces, "smooth": [bool(p.use_smooth) for p in cap.polygons],
           "sharp_edges": [list(e.vertices) for e in cap.edges if e.use_edge_sharp]}, open(OUT, "w"))
on_wall = sum(abs(sd(q)) < 0.01 * MM for q in clip)
print(f"cap: {len(clip)}-gon outline ({on_wall} vertices on the wall plane), {len(faces)} faces, "
      f"{sum(len(f) - 2 for f in faces)} tris, z {zf / MM:.3f}..{zmax / MM:.3f} mm, dropped {len(drop)} hidden faces -> {OUT}")
