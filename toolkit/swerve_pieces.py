# Swerve asset step 1: find embedded motors, split the module into pieces, dump piece facts.
#   ROBOT=<config> blender -b swerve_cad.blend --python swerve_pieces.py -- swerve_exploded.blend pieces.json
import bpy, sys, os, json, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geo, libcombo, explode
OUT, INFO = sys.argv[sys.argv.index("--") + 1:][:2]
T0 = time.time()
sc = bpy.context.scene
sw = bpy.data.objects["Swerve CAD"]
hco, htri, hmat = geo.mesh_arrays(sw.data)
motors = []
remaining = np.ones(len(htri), bool)
for mname in ("Kraken X60 CAD", "Kraken X44 CAD"):
    mo = bpy.data.objects[mname]
    pco, ptri, _ = geo.mesh_arrays(mo.data)
    for k in range(2):
        sel = remaining
        if sel.sum() < 100:
            break
        R, t, dmean, d95 = libcombo.find_embedded(pco, ptri, hco, htri[sel])
        print(f"SWP {mname} #{k}: mean {dmean*1000:.3f} mm p95 {d95*1000:.3f} mm ({time.time()-T0:.0f}s)", flush=True)
        if dmean > 0.3e-3:
            break
        placed = pco @ R.T + t
        cov = libcombo.covered_faces(hco, htri, [(placed, ptri)], tol=0.3e-3)
        remaining &= ~cov
        motors.append({"motor": mname, "R": R.tolist(), "t": t.tolist(), "mean_mm": dmean * 1000, "p95_mm": d95 * 1000, "covered_tris": int(cov.sum())})
# pieces
me = sw.data
isl = sorted(explode.islands(me), key=lambda f: -len(f))
A = explode.MeshArrays(me)
lt = A.lt
# triangle -> polygon map, to measure motor coverage per piece
me.calc_loop_triangles()
tpoly = np.empty(len(me.loop_triangles), np.int64); me.loop_triangles.foreach_get("polygon_index", tpoly)
poly_motor = np.zeros(len(me.polygons), np.float64); cnt = np.zeros(len(me.polygons))
np.add.at(poly_motor, tpoly, (~remaining).astype(float)); np.add.at(cnt, tpoly, 1)
poly_motor /= np.maximum(cnt, 1)
pieces = []
for k, f in enumerate(isl):
    pm = explode.piece_mesh(A, f, f"swerve~{k:03d}")
    po = bpy.data.objects.new(f"Swerve CAD~{k:03d}", pm)
    sc.collection.objects.link(po); po.parent = sw
    co, tri, mat = geo.mesh_arrays(pm)
    ext = np.ptp(co, axis=0) * 1000
    mfrac = float(poly_motor[f].mean())
    po["motor_frac"] = mfrac
    pieces.append({"name": po.name, "tris": int((lt[f] - 2).sum()), "ext_mm": [round(x, 1) for x in ext], "ctr_mm": [round(x, 1) for x in co.mean(0) * 1000],
                   "mats": [m.name for m in pm.materials], "motor_frac": round(mfrac, 3)})
sw.data = bpy.data.meshes.new("swerve~exploded")
mats = []
for m in me.materials:
    pb = next((n for n in (m.node_tree.nodes if m.use_nodes and m.node_tree else []) if n.bl_idname == 'ShaderNodeBsdfPrincipled'), None)
    mats.append({"name": m.name, "base": [round(x, 3) for x in pb.inputs['Base Color'].default_value[:3]] if pb else None,
                 "met": round(pb.inputs['Metallic'].default_value, 2) if pb else None, "rough": round(pb.inputs['Roughness'].default_value, 2) if pb else None,
                 "linked": bool(pb and pb.inputs['Base Color'].is_linked)})
json.dump({"motors": motors, "pieces": pieces, "materials": mats}, open(INFO, "w"), indent=0)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=False)
print(f"SWP done {len(pieces)} pieces {time.time()-T0:.0f}s", flush=True)
