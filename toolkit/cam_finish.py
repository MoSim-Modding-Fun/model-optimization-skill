# Second step for a camera asset from cam_asset.py whose lens sits in a printed hood (e.g. ELP camera mounts):
#   1. the lens is rebuilt as ONE revolve of its CAD front profile (front height per radius over the lens materials),
#      so the visible barrel lip, cone and glass dome stay round (cam_asset's per-piece Collapse faceted the barrel and
#      lathed the dome into an 8-gon);
#   2. the hood's back is closed by a flat solid cap (owner: "just make the back solid", a printed bracket sits behind
#      every mount), so no board or back plates are modelled;
#   3. faces never seen in face-id renders (culling on) are deleted again, as in cam_asset.
# Run cam_asset.py first with the hood as the CAD object, the camera as CAM_EXTRA "asset" (so the saved CAD reference
# holds the lens) and CAM_KEEP_ONLY = the hood's piece key. Then:
#   blender -b "<asset from cam_asset>.blend" --python cam_finish.py -- <out.blend> '<spec json>'
# spec: {"lens_mats": {"<cad material>": "<asset material>", ...}, "robot": "<robot>" (description), "prune": true, "axis": [x, y, z] (front, asset frame; default: the
#        area-weighted normal of the first lens material), "sides": n (default: the owner's radius tiers), "tol_mm": 0.05}
import bpy, bmesh, sys, os, json, math
import numpy as np
from mathutils import Vector
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geo, build, paths
OUT, SPEC = sys.argv[sys.argv.index("--") + 1:][:2]
spec = json.loads(SPEC)
MM = 1e-3
A = next(o for o in bpy.data.objects if o.type == 'MESH' and o.asset_data)
C = next(o for o in bpy.data.objects if o.name.endswith("(CAD)"))
me = A.data
cco, ctri, cmat = geo.mesh_arrays(C.data)
cnames = [m.name if m else "-" for m in C.data.materials]
lens_ids = [i for i, n in enumerate(cnames) if n in spec["lens_mats"]]

# ---- lens axis: through the glass centre, along its area-weighted normal (towards the front)
g = ctri[cmat == lens_ids[0]]
a, b, c = cco[g[:, 0]], cco[g[:, 1]], cco[g[:, 2]]
n_ = np.cross(b - a, c - a)
d = np.asarray(spec["axis"], float) if "axis" in spec else n_.sum(0)
d /= np.linalg.norm(d)
cen = ((a + b + c) / 3 * np.linalg.norm(n_, axis=1)[:, None]).sum(0) / np.linalg.norm(n_, axis=1).sum()
u, w = geo.perp_basis(d)

# ---- front profile: highest point per 0.1 mm radius bin over the lens materials, material of that point
P, M = [], []
for mi in lens_ids:
    pp, _ = geo.sample_surface(cco, ctri[cmat == mi], 300000, np.random.default_rng(mi))
    P.append(pp); M.append(np.full(len(pp), mi))
P = np.vstack(P); M = np.concatenate(M)
rel = P - cen
h = rel @ d
cen = cen + d * 0.0  # axis origin stays at the glass centre's foot
r = np.linalg.norm(rel - np.outer(h, d), axis=1)
BIN = 0.1 * MM
nb = int(r.max() / BIN) + 1
prof = []
for k in range(nb):
    m = (r >= k * BIN) & (r < (k + 1) * BIN)
    if m.sum() < 20:
        continue
    j = np.argmax(np.where(m, h, -1e9))
    prof.append((k * BIN + BIN / 2, h[j], M[j]))
prof = np.array(prof, dtype=object)
rr = np.array([p[0] for p in prof], float); hh = np.array([p[1] for p in prof], float); mm = [int(p[2]) for p in prof]
# the barrel wall: where the front height drops by more than 1 mm from one bin to the next
drop = np.nonzero(np.diff(hh) < -1.0 * MM)[0]
cut = int(drop[0]) + 1 if len(drop) else len(rr)
h_back = float(hh[cut]) if cut < len(rr) else float(hh.min())
rr, hh, mm = rr[:cut], hh[:cut], mm[:cut]
r_wall = float(rr[-1] + BIN / 2)
print("CAMFIN lens: glass centre", np.round(cen / MM, 2), "axis", np.round(d, 3), "r_wall %.2f mm, front %.2f..%.2f, back %.2f mm"
      % (r_wall / MM, hh.min() / MM, hh.max() / MM, h_back / MM), flush=True)
# simplify per material run (keeps the colour border exact), Douglas-Peucker on (r, h)
tol = spec.get("tol_mm", 0.05) * MM
pts = [(0.0, float(hh[0]), mm[0])]
start = 0
for i in range(1, len(rr) + 1):
    if i == len(rr) or mm[i] != mm[start]:
        seg = np.c_[rr[start:i], hh[start:i]]
        if start == 0:
            seg = np.vstack([[0.0, hh[0]], seg])
        keep = geo.douglas_peucker(seg, tol)
        for q in (keep if isinstance(keep, np.ndarray) else np.array(keep)):
            if q[0] > pts[-1][0] + 1e-7:
                pts.append((float(q[0]), float(q[1]), mm[start]))
        start = i
pts.append((r_wall, float(hh[-1]), mm[-1]))
# closed (h, r) loop for build.revolve_loop: axis pole at the front -> front profile -> wall -> back disc -> pole
loop = [(p[1], p[0]) for p in pts] + [(h_back, r_wall), (h_back, 0.0)]
loop_mat = [p[2] for p in pts] + [mm[-1], mm[-1]]
n = spec.get("sides") or geo.ngon_segments(r_wall / MM)
lens = build.revolve_loop("lens", loop, d, cen, n)
# materials per profile segment
seg = np.empty(len(lens.polygons), np.int32)
lens.attributes["seg"].data.foreach_get("value", seg) if "seg" in lens.attributes else seg.fill(0)
slot_of = {}
for mi in lens_ids:
    want = spec["lens_mats"][cnames[mi]]
    idx = next((i for i, s in enumerate(me.materials) if s and s.name == want), None)
    if idx is None:
        mat = bpy.data.materials.get(want)
        if mat is None:
            with bpy.data.libraries.load(paths.robot_materials(), link=False) as (src, dst):
                dst.materials = [want]
            mat = dst.materials[0]
        me.materials.append(mat); idx = len(me.materials) - 1
    slot_of[mi] = idx
for k, p in enumerate(lens.polygons):
    p.material_index = 0
lens_mat_idx = np.array([slot_of[loop_mat[s]] for s in seg], np.int32)
for p in lens.polygons:
    p.use_smooth = True  # glass dome and cone shade round; revolve_loop marked the profile corners > 30 deg sharp
print("CAMFIN lens revolve", n, "sides,", len(loop), "profile points,", sum(len(p.vertices) - 2 for p in lens.polygons), "tris", flush=True)

# ---- solid back: delete the hood's back-facing faces in its rearmost plane and cap its outline in that plane
bm = bmesh.new(); bm.from_mesh(me); bm.verts.ensure_lookup_table(); bm.faces.ensure_lookup_table()
D = Vector(d)
hv = np.array([(v.co - Vector(cen)).dot(D) for v in bm.verts])
h_min = float(hv.min())
back = [f for f in bm.faces if f.normal.dot(D) < -0.9 and all(hv[v.index] < h_min + 0.1 * MM for v in f.verts)]
outline = np.array([[(v.co - Vector(cen)).dot(Vector(u)), (v.co - Vector(cen)).dot(Vector(w))] for v in bm.verts if hv[v.index] < h_min + 0.1 * MM])
bmesh.ops.delete(bm, geom=back, context='FACES_ONLY')
hull = build._hull2d(outline)
base = Vector(cen) + D * h_min
# the cap is the flange's own outer boundary loop, filled (shares its vertices: a separate hull polygon leaves T-junctions
# along the rim that sparkle as see-through pixels)
pl = [e for e in bm.edges if e.is_boundary and all(hv[v.index] < h_min + 0.1 * MM for v in e.verts)]
seen_e, loops = set(), []
for e in pl:
    if e.index in seen_e:
        continue
    comp, stack = [], [e]
    while stack:
        x = stack.pop()
        if x.index in seen_e:
            continue
        seen_e.add(x.index); comp.append(x)
        stack += [y for v in x.verts for y in v.link_edges if y.is_boundary and y.index not in seen_e]
    loops.append(comp)
def _ext(comp):
    P = np.array([v.co[:] for x in comp for v in x.verts])
    return float(np.ptp(P @ np.array(u)) * np.ptp(P @ np.array(w)))
outer = max(loops, key=_ext)
res = bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=False, edges=outer)
capf = [g for g in res["geom"] if isinstance(g, bmesh.types.BMFace)]
for f in capf:
    f.normal_update()
    if f.normal.dot(D) > 0:
        f.normal_flip()
    f.material_index = 0  # the hood's own (printed) colour
    f.smooth = False
# its inner side too (0.04 mm plate): through the gap around the lens you look into the hood's cavity, and a one-sided
# cap is culled from there (a see-through spot in the gap check)
inner_v = [bm.verts.new(base + D * (0.04 * MM) + Vector(u) * float(x) + Vector(w) * float(y)) for x, y in hull]
inner = bm.faces.new(inner_v)
inner.normal_update()
if inner.normal.dot(D) < 0:
    inner.normal_flip()
inner.material_index = 0
inner.smooth = False
bmesh.ops.triangulate(bm, faces=[inner])
print("CAMFIN back cap: deleted", len(back), "back faces; rim loop", len(outer), "edges ->", len(capf), "faces; inner hull", len(hull), "points at h %.2f mm" % (h_min / MM), flush=True)
# ---- join the lens
off = len(bm.verts)
lb = bmesh.new(); lb.from_mesh(lens)
vmap = {v.index: bm.verts.new(v.co) for v in lb.verts}
for f, mi in zip(lb.faces, lens_mat_idx):
    nf = bm.faces.new([vmap[v.index] for v in f.verts])
    nf.material_index = int(mi); nf.smooth = True
sharp = {frozenset(e.vertices) for e in lens.edges if e.use_edge_sharp}
lb.free()
bm.to_mesh(me); bm.free()
for e in me.edges:
    if frozenset(e.vertices) and frozenset(v - off for v in e.vertices) in sharp and min(e.vertices) >= off:
        e.use_edge_sharp = True
bpy.data.meshes.remove(lens)
me.update()


# ---- prune faces nobody can see (copy of cam_asset.prune_hidden_faces: face-id renders, backface culling on)
def prune_hidden_faces(me, nviews=200, res=900, wd=os.path.dirname(OUT)):
    sc = bpy.context.scene
    tmp = bpy.data.objects.new("__prune", me); sc.collection.objects.link(tmp)
    for x in sc.objects:
        x.hide_render = x is not tmp
    nf = len(me.polygons)
    ids = np.repeat(np.arange(1, nf + 1), [len(p.vertices) for p in me.polygons])
    col = np.zeros((len(ids), 4), np.float32)
    col[:, 0] = ((ids & 31) * 8 + 4) / 255; col[:, 1] = (((ids >> 5) & 31) * 8 + 4) / 255
    col[:, 2] = (((ids >> 10) & 31) * 8 + 4) / 255; col[:, 3] = 1
    attr = me.color_attributes.new("__fid", 'FLOAT_COLOR', 'CORNER'); attr.data.foreach_set("color", col.ravel())
    me.color_attributes.active_color = attr
    r_ = sc.render; sh = sc.display.shading
    r_.engine = 'BLENDER_WORKBENCH'; r_.resolution_x = r_.resolution_y = res; r_.film_transparent = True; r_.dither_intensity = 0
    sc.display.render_aa = 'OFF'; sh.light = 'FLAT'; sh.color_type = 'VERTEX'; sh.show_backface_culling = True
    sh.show_cavity = sh.show_object_outline = sh.show_specular_highlight = False; sc.view_settings.view_transform = 'Raw'
    camd = bpy.data.cameras.new("__pc"); cam = bpy.data.objects.new("__pc", camd); sc.collection.objects.link(cam); sc.camera = cam
    camd.type = 'ORTHO'
    co_, _, _ = geo.mesh_arrays(me); ctr = Vector(0.5 * (co_.min(0) + co_.max(0))); R = float(np.linalg.norm(np.ptp(co_, 0))) / 2
    camd.ortho_scale = 2.04 * R; camd.clip_start = 1e-4; camd.clip_end = 8 * R
    seen = np.zeros(nf + 1, bool)
    path = os.path.join(wd, "__prune.png")
    k = np.arange(nviews) + 0.5; z = 1 - 2 * k / nviews; th = k * math.pi * (3 - math.sqrt(5))
    dirs = [Vector((math.sqrt(1 - zz * zz) * math.cos(t), math.sqrt(1 - zz * zz) * math.sin(t), zz)) for zz, t in zip(z, th)]
    dirs += [Vector(v) for v in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))]
    # + the 3x3x3 grid directions of gapcheck.py: faces seen only through the thin gap around the lens are found from
    # few directions, and a face missed there opens a see-through spot
    dirs += [Vector((x, y, z)).normalized() for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1) if (x, y, z) != (0, 0, 0)]
    for dv in dirs:
        cam.location = ctr + dv * 3 * R
        cam.rotation_euler = (-dv).to_track_quat('-Z', 'Y').to_euler()
        r_.filepath = path
        bpy.ops.render.render(write_still=True)
        img = bpy.data.images.load(path); img.colorspace_settings.name = 'Non-Color'
        px = np.empty(res * res * 4, np.float32); img.pixels.foreach_get(px); bpy.data.images.remove(img)
        px = px.reshape(-1, 4); m = px[:, 3] > 0.5
        cc = np.round(px[m, :3] * 255).astype(np.int64)
        fid = (cc[:, 0] // 8) | ((cc[:, 1] // 8) << 5) | ((cc[:, 2] // 8) << 10)
        seen[fid[fid <= nf]] = True
    me.color_attributes.remove(me.color_attributes["__fid"])
    bpy.data.objects.remove(cam); bpy.data.cameras.remove(camd); bpy.data.objects.remove(tmp)
    bm = bmesh.new(); bm.from_mesh(me); bm.faces.ensure_lookup_table()
    keep = {f.index for f in bm.faces if seen[f.index + 1]}
    keep |= {g.index for f in bm.faces if f.index in keep for e in f.edges for g in e.link_faces}  # 1-face margin
    dead = [f for f in bm.faces if f.index not in keep]
    bmesh.ops.delete(bm, geom=dead, context='FACES')
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context='VERTS')
    bm.to_mesh(me); bm.free(); me.update()
    return len(dead)


t0 = sum(len(p.vertices) - 2 for p in me.polygons)
pr = prune_hidden_faces(me) if spec.get("prune", True) else 0
t1 = sum(len(p.vertices) - 2 for p in me.polygons)
print("CAMFIN pruned", pr, "hidden faces; tris", t0, "->", t1, flush=True)
A.asset_data.description = (f"Low-poly {A.name}: printed hood + camera lens, solid back, only what is visible "
                            f"({spec.get('robot', '?')} (2026) CAD, {t1:,} tris).")
# preview: the finished asset (cam_asset's preview shows the hood before this step)
sc = bpy.context.scene
for x in bpy.data.objects:
    x.hide_render = x is not A
r_ = sc.render; sh = sc.display.shading
r_.engine = 'BLENDER_WORKBENCH'; r_.resolution_x = r_.resolution_y = 400; r_.film_transparent = True
sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'; sh.show_cavity = False; sh.show_backface_culling = True
sc.view_settings.view_transform = 'Standard'; sc.display.render_aa = '8'
camd = bpy.data.cameras.new("__pv"); cam = bpy.data.objects.new("__pv", camd); sc.collection.objects.link(cam); sc.camera = cam
camd.type = 'ORTHO'
co_, _, _ = geo.mesh_arrays(me); ctr = Vector(0.5 * (co_.min(0) + co_.max(0))); R = float(np.linalg.norm(np.ptp(co_, 0))) / 2
camd.ortho_scale = 2.1 * R; camd.clip_start = 1e-4; camd.clip_end = 8 * R
dv = (Vector(d) * 1.6 + Vector(u) * 1.0 - Vector(w) * 1.3).normalized()
cam.location = ctr + dv * 3 * R; cam.rotation_euler = (-dv).to_track_quat('-Z', 'Y').to_euler()
PREV = os.path.join(os.path.dirname(OUT), "preview.png")
r_.filepath = PREV; bpy.ops.render.render(write_still=True)
try:
    with bpy.context.temp_override(id=A):
        bpy.ops.ed.lib_id_load_custom_preview(filepath=PREV)
except Exception as ex:
    print("CAMFIN preview failed", ex, flush=True)
for x in bpy.data.objects:
    x.hide_render = False
sc = bpy.context.scene
sc.camera = None
for c_ in list(bpy.data.cameras):
    bpy.data.cameras.remove(c_)
bpy.ops.outliner.orphans_purge(do_recursive=True)
if os.path.exists(OUT):
    raise SystemExit(f"refusing to overwrite {OUT}")
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
print("CAMFIN saved", OUT, "tris", t1, flush=True)
