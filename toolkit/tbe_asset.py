# REV-11-1271 Through Bore Encoder as a library asset in the house electronics style (clean primitives, LP + Cutters ->
# one Asset object, RobotMaterials finishes linked relative, the source CAD in an excluded HP collection).
# Workflow: part_extract.py (the encoder's pieces out of the robot's exploded file; main piece regex "Solid16_2" = the
#   housing) -> tbe_asset.py -> asset_prune.py <staged> 1000 "Asset" -> gapcheck.py -> asset_finalize.py (library path,
#   Components catalog, tags, preview). Built once (886 tris, 0 gap pixels); robots fit it with LIB_ASSEMBLIES on the
#   visible shells (housing, cover, hex insert, connector), the internals are absorbed by the housing's box.
#   bl.sh tbe_src.blend --python tbe_asset.py -- <staged.blend>          (ENC_TOL outline tolerance mm, default 0.9)
# Measured from the robot CAD (REV's own model) in its own frame (z = bore axis, x toward the connector end):
#   body outline (side walls), ear plate with both ears (z 0.4..3.2), raised face region (3.2..4.0), ear bosses
#   (to 5.3 / 5.85) with their slots, back boss around the bore (-9.3..-10.0), the bore (r 10.25) with the white hex
#   insert (r 10.0, 1/2in hex 12.77 mm across flats), the connector pocket on the +x end. Hidden internals (PCB, gears,
#   magnet, bearing, screws) and the REV logo are left out. Asset frame: Z up = toward the ears' bosses, origin on the
#   bore axis at the back face.
import bpy, bmesh, sys, os, math, json
import numpy as np
from mathutils import Matrix, Vector
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import elec_asset_lib as E  # noqa: E402
import paths  # noqa: E402
a = sys.argv[sys.argv.index("--") + 1:]
OUT = a[0]; WRITE = "--write" in a
NAME = "REV-11-1271 (Through Bore Encoder)"
TOL = float(os.environ.get("ENC_TOL", "0.9"))                                     # outline tolerance (mm)
sc = bpy.context.scene
ob = lambda k: next(o for o in bpy.data.objects if k in o.name)
H, C, INS = ob("Solid16_2~"), ob("Solid5_4~"), ob("Solid3_4~")
CONN_P = ob("Solid11_3~")
VH, TH = E.tri_arrays_mm([H]); VC, TC = E.tri_arrays_mm([C]); VB, TB = E.tri_arrays_mm([H, C, CONN_P])


def sil(V, T, band, k=0):
    return E.outline(None, "z", band, res=0.05, tol=TOL, V=V, T=T)[k][1]


body = sil(VB, TB, (-4.0, -0.5))
ears = sil(VB, TB, (0.5, 0.9))
raised = sil(VH, TH, (4.1, 4.4))
outs = E.outline(None, "z", (5.1, 5.4), res=0.05, tol=TOL, V=VH, T=TH)
boss_l = next(L for A_, L in outs if L[:, 0].mean() < 0)
boss_r = sil(VH, TH, (5.5, 5.8))
back = sil(VC, TC, (-10.1, -9.7))
cover = sil(VC, TC, (-8.5, -4.4))
# slots: the two ~31.8 mm2 loops through the bosses (bmesh section of the housing at z 4.5)
slots = []
bm = bmesh.new(); bm.from_mesh(H.data)
res = bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], dist=1e-8, plane_co=(0, 0, 0.0045), plane_no=(0, 0, 1))
cut = [e for e in res["geom_cut"] if isinstance(e, bmesh.types.BMEdge)]
segs = [(np.array(e.verts[0].co[:2]) * 1000, np.array(e.verts[1].co[:2]) * 1000) for e in cut]
bm.free()
for L in E.chain(segs):
    if 25 < abs(E.area(L)) < 40:
        L = L if E.area(L) > 0 else L[::-1]
        slots.append(E.dp(L, 0.15))
# bore + hex from the insert section at z 0
bm = bmesh.new(); bm.from_mesh(INS.data)
res = bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], dist=1e-8, plane_co=(0, 0, 0), plane_no=(0, 0, 1))
segs = [(np.array(e.verts[0].co[:2]) * 1000, np.array(e.verts[1].co[:2]) * 1000) for e in res["geom_cut"] if isinstance(e, bmesh.types.BMEdge)]
bm.free()
loops = sorted(E.chain(segs), key=lambda L: abs(E.area(L)))
hexl = loops[0]
bc = hexl.mean(0)
r_corner = np.linalg.norm(hexl - bc, axis=1)
ang0 = math.atan2(*(hexl[int(np.argmax(r_corner))] - bc)[::-1])
af = math.sqrt(abs(E.area(hexl)) / (math.sqrt(3) / 2))
print(f"ENC bore centre {bc.round(2)} hex AF {af:.2f} corner angle {math.degrees(ang0):.1f}; slots {len(slots)}; outline verts "
      f"body {len(body)} ears {len(ears)} raised {len(raised)} bosses {len(boss_l)}/{len(boss_r)} back {len(back)} cover {len(cover)}", flush=True)

# ---------------------------------------------------------------- build (mm, CAD frame), then move to the asset frame
for c in ("LP", "Cutters", "Asset"):
    E.coll(c)
LP, CUT, ASSET = (bpy.data.collections[c] for c in ("LP", "Cutters", "Asset"))
L = paths.library() if hasattr(paths, "library") else os.path.dirname(paths.components())
RM = os.path.join(os.path.dirname(paths.components()), "Materials", "RobotMaterials.blend")
M = E.robot_materials(["Plastic - Black", "Plastic - White"], RM, relative=True)
BLACK, WHITE = M["Plastic - Black"], M["Plastic - White"]
CONN = WHITE          # JST-PH socket: natural white

Z_BACK, Z_COVER_TOP, Z_EAR0, Z_FACE, Z_RAISED, Z_BOSS_L, Z_BOSS_R, Z_BACKBOSS = -9.3, -4.18, 0.4, 3.2, 4.0, 5.3, 5.85, -10.0
parts = []
p_body = E.prism("enc body", body, Z_BACK, Z_EAR0 + 0.01, collection=LP)
p_ears = E.prism("enc ear plate", ears, Z_EAR0, Z_FACE, collection=LP)
p_raised = E.prism("enc raised face", raised, Z_FACE - 0.01, Z_RAISED, collection=LP)
p_bl = E.prism("enc boss L", boss_l, Z_FACE - 0.01, Z_BOSS_L, collection=LP)
p_br = E.prism("enc boss R", boss_r, Z_FACE - 0.01, Z_BOSS_R, collection=LP)
p_back = E.prism("enc back boss", back, Z_BACKBOSS, Z_BACK + 0.01, collection=LP)
for p in (p_body, p_ears, p_raised, p_bl, p_br, p_back):
    E.set_materials(p, [BLACK])
shell = E.join("enc shell", [p_body], ASSET)
for p in (p_ears, p_raised, p_bl, p_br, p_back):
    E.boolean(shell, p, 'UNION')
# cutters: bore, slots, connector pocket
R_BORE, R_INS = 10.25, 10.0
c_bore = E.prism("cut bore", E.circle(bc, R_BORE, 16), -12, 8, collection=CUT); E.set_materials(c_bore, [BLACK])
cutters = [c_bore]
for i, s in enumerate(slots):
    c = E.prism(f"cut slot {i}", s, -1, 8, collection=CUT); E.set_materials(c, [BLACK]); cutters.append(c)
c_conn = E.box("cut connector", (29.0, -16.2, -7.3), (40.0, -0.9, 0.6), collection=CUT); E.set_materials(c_conn, [BLACK, CONN])
for f_ in c_conn.data.polygons:   # pocket walls black, its floor (the cutter face looking -x) the white socket face
    f_.material_index = 1 if f_.normal.x < -0.9 else 0
cutters.append(c_conn)
for c in cutters:
    m = shell.modifiers.new("b", 'BOOLEAN'); m.operation = 'DIFFERENCE'; m.solver = 'EXACT'; m.object = c
    m.material_mode = 'TRANSFER'
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get(); dg.update()
    me = bpy.data.meshes.new_from_object(shell.evaluated_get(dg), depsgraph=dg)
    shell.modifiers.remove(m); old = shell.data; shell.data = me; bpy.data.meshes.remove(old)
# white hex insert
hexp = E.circle(bc, af / math.sqrt(3), 6, start=ang0)
ins = E.prism("enc insert", E.circle(bc, R_INS, 16), -9.72, 3.13, collection=LP); E.set_materials(ins, [WHITE])
c_hex = E.prism("cut hex", hexp, -12, 8, collection=CUT); E.set_materials(c_hex, [WHITE])
ins_a = E.join("enc insert asset", [ins], ASSET)
E.boolean(ins_a, c_hex, 'DIFFERENCE')
asset = E.join(NAME, [shell, ins_a], ASSET)
for o in (shell, ins_a):
    bpy.data.objects.remove(o)
E.tidy(asset, 1.0)
# asset frame: origin on the bore axis at the back face (z = Z_BACKBOSS), mm -> m
asset.data.transform(Matrix.Translation((-bc[0], -bc[1], -Z_BACKBOSS)))
asset.data.transform(Matrix.Scale(0.001, 4))
for o in list(LP.objects) + list(CUT.objects):
    o.data.transform(Matrix.Translation((-bc[0], -bc[1], -Z_BACKBOSS))); o.data.transform(Matrix.Scale(0.001, 4))
hp = bpy.data.collections["HP"]
for o in hp.objects:
    o.data.transform(Matrix.Translation((-bc[0] / 1000, -bc[1] / 1000, -Z_BACKBOSS / 1000)))
print(f"ENC asset tris {E.tris(asset)} open edges {E.open_edges(asset)} dims mm {[round(x * 1000, 2) for x in asset.dimensions]}", flush=True)
json.dump({"tris": E.tris(asset), "open_edges": E.open_edges(asset), "bore_centre_cad": bc.tolist(), "hex_af": af,
           "frame": "origin on the bore axis at the back face; z toward the ear bosses; x toward the connector end"},
          open(os.path.splitext(OUT)[0] + "_build.json", "w"), indent=1)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True, relative_remap=True)
print("ENC saved", OUT, flush=True)
# tri breakdown by material and facing
import collections  # noqa: E402
cnt = collections.Counter()
for p_ in asset.data.polygons:
    n = p_.normal
    d = "z" if abs(n.z) > 0.9 else "side"
    cnt[(asset.data.materials[p_.material_index].name, d)] += len(p_.vertices) - 2
print("ENC breakdown", dict(cnt), flush=True)
