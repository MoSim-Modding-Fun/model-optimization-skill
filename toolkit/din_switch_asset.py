# McMaster-Carr 8653T11 5-port DIN-rail Ethernet switch as a library asset in the style of the library's Brainbox switch
# (SW-005: bevelled case, port recesses, connector block, simplified mount; ~600 tris).
# Workflow: FRAME=normals part_extract.py (box-like part: axes from the dominant face normals, PCA is skewed by the clip)
#   -> din_switch_asset.py -> asset_prune.py -> gapcheck.py -> asset_finalize.py. The colour keys below are the CAD's
#   material names (McMaster's model); another switch model needs its own keys and checks of every printed measurement.
#   bl.sh switch_src.blend --python din_switch_asset.py -- <staged.blend>
# Measured from the robot CAD in the case's own frame (x = length, +y = the port side, z = thickness): case outline with
# its chamfered corner (rib grooves left out), the grey label panel on both big faces (0.3 mm proud), the five RJ45
# openings on +y (8 mm deep, black inside), the grey port panel, the 3-way screw terminal block on the -x end (3 square
# screw holes), the yellow DIN clip on -y. LEDs and the small white label are left out. Asset frame: Z up = thickness,
# origin at the centre of the case's bottom face, ports toward +y.
import bpy, bmesh, sys, os, math, json, collections
import numpy as np
from mathutils import Matrix, Vector
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import elec_asset_lib as E  # noqa: E402
import paths  # noqa: E402
OUT = sys.argv[sys.argv.index("--") + 1]
NAME = "8653T11 (5-Port DIN-Rail Ethernet Switch)"
src = next(o for o in bpy.data.objects if o.type == 'MESH')
BLUE, LABEL, GREY, JACK, YELLOW = ("0.000000_0.400000_0.866667", "0.733333_0.733333_0.733333", "0.643137_0.678431_0.698039",
                                   "0.933333_0.933333_0.933333", "0.933333_0.933333_0.000000")
mat = lambda key: (lambda name, n, c: np.full(len(n), name.startswith(key)))

# case: blue triangles, projected along z
V, T = E.tri_arrays_mm([src], mat(BLUE))
case = E.outline(None, "z", None, res=0.1, tol=1.0, V=V, T=T, close=8)[0][1]
zc0, zc1 = V[np.unique(T)][:, 2].min(), V[np.unique(T)][:, 2].max()
cx0, cx1 = case[:, 0].min(), case[:, 0].max(); cy0, cy1 = case[:, 1].min(), case[:, 1].max()
# label: light-grey faces looking +z on the top face
V, T = E.tri_arrays_mm([src], lambda name, n, c: name.startswith(LABEL) & (n[:, 2] > 0.9))
label = E.outline(None, "z", None, res=0.1, tol=0.6, V=V, T=T, close=4)[0][1]
# ports: openings in a y-section just inside the port face (loops of ~100-250 mm2)
bm = bmesh.new(); bm.from_mesh(src.data)
res_ = bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], dist=1e-8, plane_co=(0, (cy1 - 1.1) / 1000, 0), plane_no=(0, 1, 0))
segs = [(np.array((e.verts[0].co.x, e.verts[0].co.z)) * 1000, np.array((e.verts[1].co.x, e.verts[1].co.z)) * 1000)
        for e in res_["geom_cut"] if isinstance(e, bmesh.types.BMEdge)]
bm.free()
ports = []
for L in E.chain(segs):
    A_ = abs(E.area(L))
    w_, h_ = np.ptp(L[:, 0]), np.ptp(L[:, 1])
    if 80 < A_ < 140 and 8 < w_ < 14 and 8 < h_ < 14:
        ports.append((L[:, 0].min(), L[:, 0].max(), L[:, 1].min(), L[:, 1].max(), E.dp(L if E.area(L) > 0 else L[::-1], 0.3)))
ports.sort()
# port panel: grey faces looking +y
V, T = E.tri_arrays_mm([src], lambda name, n, c: name.startswith(GREY) & (n[:, 1] > 0.9) & (c[:, 1] > cy1 - 3))
pp = V[np.unique(T)] if len(T) else None
panel = (pp[:, 0].min(), pp[:, 0].max(), pp[:, 2].min(), pp[:, 2].max()) if pp is not None else None
# terminal block: grey beyond the -x end of the case
V, T = E.tri_arrays_mm([src], lambda name, n, c: name.startswith(GREY) & (c[:, 0] < cx0 + 0.5))
tb = V[np.unique(T)]
tb_lo, tb_hi = tb.min(0), tb.max(0)
# terminal screw holes: small closed loops of a z-section through the block (they run along z)
bm = bmesh.new(); bm.from_mesh(src.data)
zmid = 0.5 * (tb_lo[2] + tb_hi[2]) / 1000
res_ = bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], dist=1e-8, plane_co=(0, 0, zmid), plane_no=(0, 0, 1))
segs = [(np.array(e.verts[0].co[:2]) * 1000, np.array(e.verts[1].co[:2]) * 1000) for e in res_["geom_cut"]
        if isinstance(e, bmesh.types.BMEdge) and min(e.verts[0].co.x, e.verts[1].co.x) * 1000 < cx0 - 0.5]
bm.free()
holes = []
for L in E.chain(segs):
    if 2 < abs(E.area(L)) < 40 and L[:, 0].max() < cx0:
        holes.append(E.dp(L if E.area(L) > 0 else L[::-1], 0.2))
holes.sort(key=lambda L: L[:, 1].mean())
# DIN clip: yellow, projected along y (x, z), y range
V, T = E.tri_arrays_mm([src], mat(YELLOW))
clip = E.outline(None, "y", None, res=0.1, tol=1.0, V=V, T=T, close=6)[0][1]
cy = V[np.unique(T)][:, 1]
print(f"SW case x {cx0:.2f}..{cx1:.2f} y {cy0:.2f}..{cy1:.2f} z {zc0:.2f}..{zc1:.2f} ({len(case)} verts); label {len(label)} verts "
      f"x {label[:,0].min():.1f}..{label[:,0].max():.1f}; ports {[tuple(round(float(v), 1) for v in p[:4]) for p in ports]}; panel {panel}; "
      f"terminal {tb_lo.round(1)}..{tb_hi.round(1)}; holes {len(holes)} {[(round(float(L[:,0].mean()),1), round(float(L[:,1].mean()),1), round(abs(E.area(L)),1)) for L in holes][:6]}; "
      f"clip {len(clip)} verts y {cy.min():.2f}..{cy.max():.2f}", flush=True)

# ---------------------------------------------------------------- build
for c in ("LP", "Cutters", "Asset"):
    E.coll(c)
LP, CUT, ASSET = (bpy.data.collections[c] for c in ("LP", "Cutters", "Asset"))
RM = os.path.join(os.path.dirname(paths.components()), "Materials", "RobotMaterials.blend")
M = E.robot_materials(["Plastic - Blue", "Plastic - Grey 50", "Plastic - Black", "Plastic - CANYellow"], RM)
MB, MG, MK, MY = M["Plastic - Blue"], M["Plastic - Grey 50"], M["Plastic - Black"], M["Plastic - CANYellow"]
body = E.prism("sw case", case, zc0, zc1, collection=LP); E.set_materials(body, [MB])
shell = E.join("sw shell", [body], ASSET)
lab_t = E.prism("sw label top", label, zc1 - 0.01, zc1 + 0.3, collection=LP); E.set_materials(lab_t, [MG])
lab_b = E.prism("sw label bottom", label, zc0 - 0.3, zc0 + 0.01, collection=LP); E.set_materials(lab_b, [MG])
# port panel: grey plate 0.3 mm proud on +y
parts = [lab_t, lab_b]
if panel:
    pnl = E.prism("sw port panel", np.array([[panel[0], panel[2]], [panel[1], panel[2]], [panel[1], panel[3]], [panel[0], panel[3]]]),
                  cy1 + 0.3, cy1 - 0.01, collection=LP, axis="y")
    E.set_materials(pnl, [MG]); parts.append(pnl)
tblk = E.box("sw terminal block", (tb_lo[0], tb_lo[1], tb_lo[2]), (cx0 + 0.5, tb_hi[1], tb_hi[2]), collection=LP); E.set_materials(tblk, [MG])
parts.append(tblk)
clp = E.prism("sw din clip", clip, cy0 + 0.3, cy.min(), collection=LP, axis="y"); E.set_materials(clp, [MY]); parts.append(clp)
for p in parts:
    E.boolean(shell, p, 'UNION')
# cutters: RJ45 openings (rectangle + key notch on the latch side), terminal screw holes
cut = []
for i, (x0, x1, z0, z1, P) in enumerate(ports):
    c = E.prism(f"cut rj45 {i}", P, cy1 + 1.0, cy1 - 6.0, collection=CUT, axis="y"); E.set_materials(c, [MK]); cut.append(c)
for i, L in enumerate(holes[:3]):
    c = E.prism(f"cut screw {i}", L, tb_hi[2] - 3.0, tb_hi[2] + 1.0, collection=CUT); E.set_materials(c, [MK]); cut.append(c)
for c in cut:
    m = shell.modifiers.new("b", 'BOOLEAN'); m.operation = 'DIFFERENCE'; m.solver = 'EXACT'; m.object = c; m.material_mode = 'TRANSFER'
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get(); dg.update()
    me = bpy.data.meshes.new_from_object(shell.evaluated_get(dg), depsgraph=dg)
    shell.modifiers.remove(m); old = shell.data; shell.data = me; bpy.data.meshes.remove(old)
shell.name = NAME
E.tidy(shell, 1.0)
off = Vector((-(cx0 + cx1) / 2, -(cy0 + cy1) / 2, -(zc0 - 0.3)))
for o in list(LP.objects) + list(CUT.objects) + [shell]:
    o.data.transform(Matrix.Translation(off)); o.data.transform(Matrix.Scale(0.001, 4))
for o in bpy.data.collections["HP"].objects:
    o.data.transform(Matrix.Translation(off / 1000))
cnt = collections.Counter()
for p_ in shell.data.polygons:
    cnt[shell.data.materials[p_.material_index].name] += len(p_.vertices) - 2
print(f"SW asset tris {E.tris(shell)} open edges {E.open_edges(shell)} dims mm {[round(x * 1000, 2) for x in shell.dimensions]} by finish {dict(cnt)}", flush=True)
json.dump({"tris": E.tris(shell), "open_edges": E.open_edges(shell), "frame": "origin at the centre of the case's bottom face (labels' "
           "outer face at z=0); +y = ports; x = length (terminal block at -x)"}, open(os.path.splitext(OUT)[0] + "_build.json", "w"), indent=1)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True, relative_remap=True)
print("SW saved", OUT, flush=True)
