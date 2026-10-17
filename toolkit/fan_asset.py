# Low-poly 40x40x10 mm cooling fan as a library asset, in the style of the team's electronics assets (one mesh object
# at the origin, centred, RobotMaterials finish, CAD reference kept in the file but out of the view layer).
#   blender -b "<robot> - cleaned.blend" --python fan_asset.py -- <cad object name> <out.blend> "<asset name>" <preview.png|->
# Frame: rounded-square prism with the venturi (24 sides) and 4 mounting holes (8 sides); hub: 16-sided cylinder;
# 8 pitched blades (3 stations each); 4 motor struts on the back. Airflow axis = Z, label/hub face = +Z.
import bpy, bmesh, sys, os, math
import numpy as np
from mathutils import Vector, Matrix
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build, geo, libcombo
CAD, OUT, NAME, PREVIEW = sys.argv[sys.argv.index("--") + 1:][:4]
import paths  # noqa: E402
RM = paths.robot_materials()
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"  # same catalog as the other electronics assets
MM = 1e-3
S, T = 40 * MM, 10 * MM            # frame size, thickness
R_CORNER = 3.0 * MM
R_VENTURI = 19.0 * MM
HOLE_C, R_HOLE = 16.0 * MM, 1.7 * MM
R_HUB, HUB_Z = 10.5 * MM, (-4.75 * MM, 4.75 * MM)
N_BLADES, R_ROOT, R_TIP = 8, 10.2 * MM, 18.6 * MM
X, Y, Z = Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1))


def circle(cx, cy, r, n, phase=0.0):
    return np.array([(cx + r * math.cos(phase + 2 * math.pi * k / n), cy + r * math.sin(phase + 2 * math.pi * k / n)) for k in range(n)])


def rounded_square(half, rc, per_corner=3):
    pts = []
    for cx, cy, a0 in ((half - rc, half - rc, 0), (-half + rc, half - rc, 90), (-half + rc, -half + rc, 180), (half - rc, -half + rc, 270)):
        for k in range(per_corner + 1):
            a = math.radians(a0 + 90 * k / per_corner)
            pts.append((cx + rc * math.cos(a), cy + rc * math.sin(a)))
    return np.array(pts)


parts = []
# frame with venturi + mounting holes
holes = [circle(0, 0, R_VENTURI, geo.ngon_segments(R_VENTURI / MM), math.pi / 24)]
holes += [circle(sx * HOLE_C, sy * HOLE_C, R_HOLE, geo.ngon_segments(R_HOLE / MM)) for sx in (1, -1) for sy in (1, -1)]
parts.append(build.build_prism("frame", rounded_square(S / 2, R_CORNER), holes, X, Y, Z, -T / 2, T / 2))
# hub (closed at both faces)
parts.append(build.build_prism("hub", circle(0, 0, R_HUB, geo.ngon_segments(R_HUB / MM)), [], X, Y, Z, *HUB_Z))


def blade(phi0):
    """Thin pitched blade from root to tip; 3 radial stations, swept forward, chord grows toward the tip."""
    verts, faces = [], []
    # measured on the CAD: blades cover 76% of the circle at r 12 mm and 61% at r 17 mm over 6.9 mm of depth
    stations = [(R_ROOT, 10.0 * MM, 45), (0.5 * (R_ROOT + R_TIP), 10.3 * MM, 42), (R_TIP, 10.8 * MM, 38)]
    for i, (r, chord, pitch) in enumerate(stations):
        phi = phi0 + math.radians(5) * i  # slight forward sweep
        radial = Vector((math.cos(phi), math.sin(phi), 0))
        tang = Vector((-math.sin(phi), math.cos(phi), 0))
        a = math.radians(pitch)
        cdir = tang * math.cos(a) + Z * math.sin(a)          # chord direction
        ndir = cdir.cross(radial).normalized() * 0.45 * MM    # half thickness
        c = radial * r + Z * 0.45 * MM
        for s1, s2 in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            verts.append(c + cdir * (s1 * chord / 2) + ndir * s2)
    for i in range(2):
        b0, b1 = 4 * i, 4 * (i + 1)
        for k in range(4):
            k2 = (k + 1) % 4
            faces.append((b0 + k, b0 + k2, b1 + k2, b1 + k))
    faces.append((3, 2, 1, 0))
    faces.append((8, 9, 10, 11))
    me = build.new_mesh_from("blade", [tuple(v) for v in verts], faces)
    build.fix_normals(me)
    return me


for k in range(N_BLADES):
    parts.append(blade(2 * math.pi * k / N_BLADES))


def bar(a, w, t, r0, r1, z0):
    """Strut on the back face: box from r0 to r1 along angle a, width w, thickness t starting at z0."""
    d = Vector((math.cos(a), math.sin(a), 0)); n = Vector((-math.sin(a), math.cos(a), 0))
    verts = [d * r + n * (s * w / 2) + Z * zz for zz in (z0, z0 + t) for r in (r0, r1) for s in (-1, 1)]
    faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    me = build.new_mesh_from("strut", [tuple(v) for v in verts], faces)
    build.fix_normals(me)
    return me


for ang in (47, 74, 187, 310):  # strut angles measured on the CAD's back face
    parts.append(bar(math.radians(ang), 2.2 * MM, 1.6 * MM, R_HUB - 0.5 * MM, R_VENTURI + 0.3 * MM, -T / 2))

# one mesh, one finish
with bpy.data.libraries.load(RM, link=False) as (src, dst):
    dst.materials = ["Plastic - Black"]
black = dst.materials[0]
for m in parts:
    m.materials.clear(); m.materials.append(black)
me = libcombo.join_meshes(NAME, [(m, Matrix.Identity(4)) for m in parts])
for m in parts:
    bpy.data.meshes.remove(m)
me.name = NAME

# fit check against the CAD fan (CAD local frame: z 0..10 mm -> shift by -5 mm)
cad = bpy.data.objects[CAD]
cco, ctri, _ = geo.mesh_arrays(cad.data)
cco = cco - np.array([0, 0, 5 * MM])
aco, atri, _ = geo.mesh_arrays(me)
import align
A, _ = geo.sample_surface(aco, atri, 4000); B, _ = geo.sample_surface(cco, ctri, 15000)
ch = align.chamfer(A, B, np.eye(3), np.zeros(3))
tris = sum(len(p.vertices) - 2 for p in me.polygons)
print("FAN asset tris", tris, "chamfer mm (asset->CAD, CAD->asset)", [round(c * 1000, 2) for c in ch], flush=True)

# new file content: only the asset + the CAD reference
cad_me = cad.data.copy(); cad_me.transform(Matrix.Translation((0, 0, -5 * MM)))
for o in list(bpy.data.objects):
    bpy.data.objects.remove(o)
for c in list(bpy.data.collections):
    bpy.data.collections.remove(c)
sc = bpy.context.scene
ob = bpy.data.objects.new(NAME, me)
sc.collection.objects.link(ob)
ref = bpy.data.collections.new("CAD reference")
sc.collection.children.link(ref)
ro = bpy.data.objects.new(NAME + " (CAD)", cad_me)
ref.objects.link(ro)
bpy.context.view_layer.layer_collection.children["CAD reference"].exclude = True
ob.asset_mark()
ob.asset_data.catalog_id = CATALOG
if PREVIEW != "-" and os.path.exists(PREVIEW):
    try:
        with bpy.context.temp_override(id=ob):
            bpy.ops.ed.lib_id_load_custom_preview(filepath=PREVIEW)
        print("FAN preview loaded", flush=True)
    except Exception as ex:
        print("FAN preview failed", ex, flush=True)
bpy.ops.outliner.orphans_purge(do_recursive=True)
bpy.context.preferences.filepaths.save_version = 0
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
print("FAN saved", OUT, flush=True)
