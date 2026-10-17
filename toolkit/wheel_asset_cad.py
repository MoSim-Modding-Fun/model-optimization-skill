# Compliant wheel library asset in the style of the owner's am-3562 file: a solid low-side cylinder (rubber) with a
# solid black hub cap, the windows cut by one cutter prism arrayed around an Empty and a Boolean (Fast), then Smooth
# by Angle; blue 50A and green 35A variants; collections HP (CAD), LP (live Boolean), Cutters, Asset (applied).
#   blender -b "<robot CAD>.blend" --python wheel_asset_cad.py -- "<wheel part>" "<hub part|->" <out.blend> <prefix> \
#       "<size label>" <sides> <preview dir|-> ["<asset name suffix>"]
# e.g. ... -- "am-3945 3IN 500Hex Compliant Wheel REV 3" - ".../am-3945 (...).blend" am-3945 "3in OD 1in WD" 16 /tmp/pv
# Dimensions, window count and window shape come from the CAD's mid-width section. The hub cap covers the CAD
# overmold's bore (separate polycarbonate hub); without one ("-" and no bore loop) the wheel is one rubber piece.
# Axis = Z, centred at the origin, one window centred on +Y (as am-3562).
import bpy, bmesh, sys, os, re, math
import numpy as np
from mathutils import Vector, Matrix
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build, geo, shading, align
ARGS = sys.argv[sys.argv.index("--") + 1:]
WHEEL, HUB, OUT, PREFIX, LABEL, SIDES, PREVIEW = ARGS[:7]
SUFFIX = ARGS[7] if len(ARGS) > 7 else ""  # tells two library versions of one wheel apart (", 16-sided")
SIDES = int(SIDES)
import paths  # noqa: E402
RM = paths.robot_materials()
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"  # am-3562's catalog
VARIANTS = [("green", "35A", "Rubber - Green"), ("blue", "50A", "Rubber - Blue")]  # am-3562 ships these two
MM = 1e-3
CUT_MARGIN = 3.175 * MM  # am-3562's cutter sticks out 1/8" past each face
MAX_WIN_VERTS = 12  # am-3562's window cutter has 12 vertices
part = lambda n: re.sub(r"(\.\d{3})+$", "", n)
sc = bpy.context.scene

# ------------------------------------------------------------------ CAD measurements
cands = [o for o in sc.objects if o.type == 'MESH' and part(o.name) == WHEEL]
wo = max(cands, key=lambda o: len(o.data.polygons))
me = wo.data
co, tri, _ = geo.mesh_arrays(me)
pts, _ = geo.sample_surface(co, tri, 20000)
d = min(((geo.revolve_score(pts, a), a) for a in geo.candidate_axes(pts)), key=lambda x: x[0][0])[1]
d = d / np.linalg.norm(d)
u, _ = geo.perp_basis(d)
u = u / np.linalg.norm(u)
w = np.cross(d, u)  # right-handed (u, w, d)
h = co @ d
H0, H1 = float(h.min()), float(h.max())
WIDTH = H1 - H0
loops, lu, lw = build.section_loops(me, d, (H0 + H1) / 2)
# section_loops' own basis -> ours
B = np.array([lu, lw])
loops = [L @ B @ np.c_[u, w] for L in loops if abs(geo.poly_area(L)) > (0.3 * MM) ** 2]
outer = max(loops, key=lambda L: abs(geo.poly_area(L)))
c2 = (outer.max(0) + outer.min(0)) / 2
R_OUT = float(np.linalg.norm(outer - c2, axis=1).max())
centre3 = c2[0] * u + c2[1] * w + d * (H0 + H1) / 2
bore = [L for L in loops if L is not outer and build.parity_inside(c2[None], [L])[0]]
R_HUB = float(np.linalg.norm(bore[0] - c2, axis=1).mean()) if bore and HUB != "-" else 0.0  # one-piece: solid rubber
def centroid(L):
    """Area centroid (the vertex mean leans toward densely tessellated corners)."""
    x, y = L[:, 0], L[:, 1]
    cr = x * np.roll(y, -1) - np.roll(x, -1) * y
    a = cr.sum() / 2
    return np.array([((x + np.roll(x, -1)) * cr).sum(), ((y + np.roll(y, -1)) * cr).sum()]) / (6 * a)


info = []
for L in loops:
    if L is outer or any(L is b for b in bore):
        continue
    m = centroid(L)
    info.append((L, abs(geo.poly_area(L)), float(np.linalg.norm(m - c2)), math.atan2(m[1] - c2[1], m[0] - c2[0])))
best = []
for _, a, r, _ in info:
    grp = [x for x in info if abs(x[1] - a) < 0.03 * a and abs(x[2] - r) < 1 * MM]
    if len(grp) > len(best):
        best = grp
ang = sorted(x[3] % (2 * math.pi) for x in best)
step = min((ang[(k + 1) % len(ang)] - ang[k]) % (2 * math.pi) for k in range(len(ang)))
N_WIN = round(2 * math.pi / step)
tmpl = max(best, key=lambda x: len(x[0]))
print("WHEEL CAD", WHEEL, "OD mm", round(2 * R_OUT / MM, 2), "width", round(WIDTH / MM, 2), "hub r", round(R_HUB / MM, 2),
      "windows", N_WIN, "(", len(best), "clean in the CAD )", flush=True)

# frame: window centroid on +Y, axis on Z, centre at the origin; the mirror axis is refined within +-2 deg
def seg_dist(P, L):
    """Distance from each point of P to the closed polyline L."""
    A, Bq = L, np.roll(L, -1, axis=0)
    AB = Bq - A
    t = np.clip(((P[:, None, :] - A[None]) * AB[None]).sum(-1) / np.maximum((AB ** 2).sum(-1), 1e-18)[None], 0, 1)
    Q = A[None] + t[..., None] * AB[None]
    return np.linalg.norm(P[:, None, :] - Q, axis=-1).min(1)


def placed(rot):
    Rz = np.array([[math.cos(rot), -math.sin(rot)], [math.sin(rot), math.cos(rot)]])
    return (tmpl[0] - c2) @ Rz.T


rot = min((math.pi / 2 - tmpl[3] + math.radians(x) for x in np.linspace(-2, 2, 41)),
          key=lambda r: np.percentile(seg_dist(placed(r) * np.array([-1, 1]), placed(r)), 95))
win = placed(rot)
TO_ASSET = Matrix([[*u, 0], [*w, 0], [*d, 0], [0, 0, 0, 1]])
TO_ASSET = Matrix.Rotation(rot, 4, 'Z') @ TO_ASSET @ Matrix.Translation(-Vector(centre3))


# windows are mirror-symmetric about their radial line in the CAD: simplify one half and mirror it, so the cutter
# is exactly symmetric (Douglas-Peucker on the whole loop leans to one side depending on where it starts)
mir = win * np.array([-1, 1])
sym_err = float(np.percentile(seg_dist(mir, win), 95))
symmetric = sym_err < 0.3 * MM
print("WHEEL window mirror error p95 mm", round(sym_err / MM, 3), flush=True)
if geo.poly_area(win) < 0:
    win = win[::-1]


def half_chain(L):
    """x >= 0 half of a CCW loop symmetric about the y axis, from its bottom crossing to its top crossing."""
    n = len(L)
    cross = []
    for i in range(n):
        a, b = L[i], L[(i + 1) % n]
        if (a[0] < 0) != (b[0] < 0):
            t = a[0] / (a[0] - b[0])
            cross.append((i, a + t * (b - a)))
    lo = min(cross, key=lambda x: x[1][1])
    hi = max(cross, key=lambda x: x[1][1])
    chain, i = [np.array([0.0, lo[1][1]])], (lo[0] + 1) % n
    while True:
        if L[i][0] >= 0:
            chain.append(L[i])
        if i == hi[0]:
            break
        i = (i + 1) % n
    chain.append(np.array([0.0, hi[1][1]]))
    return np.array(chain)


tol = 0.15 * MM
while True:
    if symmetric:
        hc = geo.douglas_peucker(half_chain(win), tol, closed=False)
        prof = np.vstack([hc, (hc[1:-1] * np.array([-1, 1]))[::-1]])
    else:
        prof = geo.douglas_peucker(build._canonical_loop(win), tol, closed=True)
    if len(prof) <= MAX_WIN_VERTS or tol > 2 * MM:
        break
    tol *= 1.25
win_err = float(np.percentile(seg_dist(win, prof), 95))
print("WHEEL window profile", len(prof), "verts, symmetric", bool(symmetric), "tol mm", round(tol / MM, 2),
      "p95 err mm", round(win_err / MM, 2), flush=True)

# ------------------------------------------------------------------ new file content
cad_meshes = []
for o in [wo] + ([min((x for x in sc.objects if x.type == 'MESH' and part(x.name) == HUB),
                      key=lambda x: (x.matrix_world.translation - wo.matrix_world.translation).length)] if HUB != "-" else []):
    m = o.data.copy()
    m.transform(TO_ASSET @ wo.matrix_world.inverted() @ o.matrix_world)
    cad_meshes.append((o.name, m))
for o in list(bpy.data.objects):
    bpy.data.objects.remove(o)
for c in list(bpy.data.collections):
    bpy.data.collections.remove(c)
colls = {}
for n in ("HP", "LP", "Asset", "Cutters"):  # am-3562's order
    colls[n] = bpy.data.collections.new(n)
    sc.collection.children.link(colls[n])
for n, m in cad_meshes:
    o = bpy.data.objects.new(n, m)
    colls["HP"].objects.link(o)

with bpy.data.libraries.load(RM, link=True, relative=True) as (src, dst):
    dst.materials = [v[2] for v in VARIANTS] + ["Plastic - Black"]
mats = {m.name: m for m in dst.materials}


def cylinder_mesh(name):
    """Solid N-sided disc; with a hub, the faces inside R_HUB are the black hub cap (am-3562: 12-gon hub faces
    joined to the outer ring by quads). Flats at the true radius, so the tread still touches where the wheel does."""
    bm = bmesh.new()
    k = 1 / math.cos(math.pi / SIDES)
    ring = lambda r, z: [bm.verts.new((r * k * math.cos(2 * math.pi * i / SIDES), r * k * math.sin(2 * math.pi * i / SIDES), z))
                         for i in range(SIDES)]
    zt, zb = WIDTH / 2, -WIDTH / 2
    ot, ob = ring(R_OUT, zt), ring(R_OUT, zb)
    for i in range(SIDES):
        j = (i + 1) % SIDES
        bm.faces.new((ob[i], ob[j], ot[j], ot[i])).material_index = 0
    if R_HUB > 0:
        ht, hb = ring(R_HUB, zt), ring(R_HUB, zb)
        for i in range(SIDES):
            j = (i + 1) % SIDES
            bm.faces.new((ht[i], ht[j], ot[j], ot[i])).material_index = 0
            bm.faces.new((hb[j], hb[i], ob[i], ob[j])).material_index = 0
        bm.faces.new(ht).material_index = 1
        bm.faces.new(hb[::-1]).material_index = 1
    else:
        bm.faces.new(ot).material_index = 0
        bm.faces.new(ob[::-1]).material_index = 0
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)  # closed solid: outward everywhere
    m = bpy.data.meshes.new(name)
    bm.to_mesh(m)
    bm.free()
    return m


# cutter: one window prism, arrayed around an Empty (am-3562: Cube + Array(object offset = Empty))
bm = bmesh.new()
zc = WIDTH / 2 + CUT_MARGIN
top = [bm.verts.new((x, y, zc)) for x, y in prof]
bot = [bm.verts.new((x, y, -zc)) for x, y in prof]
bm.faces.new(top)
bm.faces.new(bot[::-1])
for i in range(len(prof)):
    j = (i + 1) % len(prof)
    bm.faces.new((bot[i], bot[j], top[j], top[i]))
bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
cm = bpy.data.meshes.new("Cube")
bm.to_mesh(cm)
bm.free()
empty = bpy.data.objects.new("Empty", None)
empty.rotation_euler = (0, 0, 2 * math.pi / N_WIN)
colls["LP"].objects.link(empty)
cutter = bpy.data.objects.new("Cube", cm)
colls["Cutters"].objects.link(cutter)
cutter.display_type = 'WIRE'
cutter.hide_render = True
arr = cutter.modifiers.new("Array", 'ARRAY')
arr.fit_type, arr.count = 'FIXED_COUNT', N_WIN
arr.use_relative_offset, arr.use_object_offset, arr.offset_object = False, True, empty
sba = shading.sba_group()
cutter.modifiers.new("Smooth by Angle", 'NODES').node_group = sba

dg = bpy.context.evaluated_depsgraph_get()
assets = []
for vi, (col, duro, rub) in enumerate(VARIANTS):
    lp = bpy.data.objects.new("Cylinder" if vi == 0 else f"Cylinder.{vi:03d}", cylinder_mesh(f"Cylinder.{vi:03d}"))
    colls["LP"].objects.link(lp)
    lp.data.materials.append(mats[rub])
    lp.data.materials.append(mats["Plastic - Black"])
    for p in lp.data.polygons:
        p.use_smooth = True
    bo = lp.modifiers.new("Boolean", 'BOOLEAN')
    bo.operation, bo.operand_type, bo.object, bo.solver = 'DIFFERENCE', 'OBJECT', cutter, 'FLOAT'
    lp.modifiers.new("Smooth by Angle", 'NODES').node_group = sba
    dg.update()
    am = bpy.data.meshes.new_from_object(lp.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
    name = f"{PREFIX}_{col} ({LABEL} {duro} Compliant Wheel{SUFFIX})"
    am.name = name
    ao = bpy.data.objects.new(name, am)
    colls["Asset"].objects.link(ao)
    assets.append(ao)

# ------------------------------------------------------------------ checks
am = assets[0].data
tris = sum(len(p.vertices) - 2 for p in am.polygons)
aco, atri, _ = geo.mesh_arrays(am)
bco = np.vstack([geo.mesh_arrays(m)[0] for _, m in cad_meshes])
btri = np.vstack([geo.mesh_arrays(m)[1] + off for (_, m), off in
                  zip(cad_meshes, np.cumsum([0] + [len(m.vertices) for _, m in cad_meshes[:-1]]))])
A, _ = geo.sample_surface(aco, atri, 6000)
Bs, _ = geo.sample_surface(bco, btri, 20000)
ch = align.chamfer(A, Bs, np.eye(3), np.zeros(3))
sec, _, _ = build.section_loops(am, np.array([0, 0, 1.0]), 0.0)
holes = sum(1 for L in sec if not build.parity_inside(np.zeros((1, 2)), [L])[0] and abs(geo.poly_area(L)) < 0.5 * math.pi * R_OUT ** 2)
print("WHEEL asset", name, "tris", tris, "open edges", build.open_edges(am), "windows cut", holes, "of", N_WIN,
      "chamfer mm", [round(x / MM, 2) for x in ch], flush=True)

for ao in assets:
    ao.asset_mark()
    ao.asset_data.catalog_id = CATALOG

# previews: Workbench render with local stand-ins for the linked materials (linked ones can't take a viewport colour)
if PREVIEW != "-":
    os.makedirs(PREVIEW, exist_ok=True)
    sc.render.engine = 'BLENDER_WORKBENCH'
    sc.render.resolution_x = sc.render.resolution_y = 256
    sc.render.film_transparent = True
    shd = sc.display.shading
    shd.light, shd.color_type, shd.show_cavity = 'STUDIO', 'MATERIAL', False
    sc.display.render_aa = '8'
    cam = bpy.data.objects.new("__pcam", bpy.data.cameras.new("__pcam"))
    sc.collection.objects.link(cam)
    sc.camera = cam
    cam.data.type, cam.data.ortho_scale = 'ORTHO', 2.6 * R_OUT
    v = Vector((0.55, -0.75, 0.75)).normalized()
    cam.location = v * 6 * R_OUT
    cam.rotation_euler = (-v).to_track_quat('-Z', 'Y').to_euler()
    for ao in assets:
        tmp = ao.data.copy()
        for i, m in enumerate(tmp.materials):
            lm = m.copy()
            bsdf = lm.node_tree.nodes.get("Principled BSDF")
            lm.diffuse_color = bsdf.inputs["Base Color"].default_value
            tmp.materials[i] = lm
        po = bpy.data.objects.new("__prev", tmp)
        sc.collection.objects.link(po)
        for o in sc.objects:
            o.hide_render = o is not po
        png = os.path.join(PREVIEW, ao.name + ".png")
        sc.render.filepath = png
        bpy.ops.render.render(write_still=True)
        bpy.data.objects.remove(po)
        try:
            with bpy.context.temp_override(id=ao):
                bpy.ops.ed.lib_id_load_custom_preview(filepath=png)
        except Exception as ex:
            print("WHEEL preview failed", ex, flush=True)
    bpy.data.objects.remove(cam)
    for o in sc.objects:
        o.hide_render = o is cutter
    sc.render.film_transparent = False

for o in [x for x in bpy.data.objects if x.name.startswith("__")]:
    bpy.data.objects.remove(o)
bpy.ops.outliner.orphans_purge(do_recursive=True)
bpy.context.preferences.filepaths.save_version = 0
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True, relative_remap=True)
print("WHEEL saved", OUT, [a.name for a in assets], flush=True)
