# Swerve asset step 2: drop the reference Krakens, fit the library motors to the swerve's motor pieces.
#   ROBOT=<config> blender -b swerve_exploded.blend --python swerve_prep.py -- swerve_work.blend motors.json
import bpy, sys, os, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geo, align
OUT, MOT = sys.argv[sys.argv.index("--") + 1:][:2]
for n in ("Kraken X60 CAD", "Kraken X44 CAD"):
    bpy.data.objects.remove(bpy.data.objects[n])
LIBS = {"Kraken X44": ("WCP-0941 (Kraken X44)/WCP-0941 (Kraken X44).blend", "WCP-0941 (Kraken X44)"),
        "Kraken X60": ("WCP-0940 (Kraken X60)/WCP-0940 (Kraken X60).blend", "WCP-0940 (Kraken X60)")}
res = {}
for piece, kind in (("Swerve CAD~000", "Kraken X44"), ("Swerve CAD~001", "Kraken X60")):
    f, a = LIBS[kind]
    ob = align.append_object(f"{align.LIB}/{f}", a)
    bpy.context.scene.collection.objects.link(ob)
    dg = bpy.context.evaluated_depsgraph_get()
    lm = bpy.data.meshes.new_from_object(ob.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
    lm.transform(ob.matrix_world)
    bpy.data.objects.remove(ob)
    sco, stri, _ = geo.mesh_arrays(lm)
    dco, dtri, _ = geo.mesh_arrays(bpy.data.objects[piece].data)
    R, t, ch = align.align(sco, stri, dco, dtri)
    res[piece] = {"kind": kind, "lib": [f, a], "R": np.asarray(R).tolist(), "t": np.asarray(t).tolist(), "chamfer_mm": [round(c * 1000, 2) for c in ch], "lib_tris": sum(len(p.vertices) - 2 for p in lm.polygons)}
    print("PREP", piece, kind, "chamfer mm", res[piece]["chamfer_mm"], "lib tris", res[piece]["lib_tris"], flush=True)
    bpy.data.meshes.remove(lm)
json.dump(res, open(MOT, "w"), indent=1)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=False)
