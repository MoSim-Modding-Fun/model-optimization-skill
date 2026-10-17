# Prep a robot source for the pipeline: dump what later checks compare against, weld every mesh, split merged
# sub-assemblies. The source file itself is never saved (explode.py writes the exploded copy).
#   ROBOT=<config> blender -b "<source>.blend" --python prep.py -- <work dir>
# Writes into <work dir>: orig_names.json (every object name: verify.py), tf_src.json (every object's world matrix:
# tfcheck.py), cad_mats.json (the CAD's material names: phase3.py --cad-mats), then exploded.blend + explode.json.
# Weld distance: cfg.PRE_WELD_M (default 1e-6 m; glTF exports are often polygon soup, and everything that walks edges
# fails on it). With no cfg.EXPLODE the exploded file is just the welded copy.
import bpy, sys, os, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import robots, build
cfg = robots.load()
W = sys.argv[sys.argv.index("--") + 1]
os.makedirs(W, exist_ok=True)
json.dump(sorted(o.name for o in bpy.data.objects), open(os.path.join(W, "orig_names.json"), "w"))
json.dump({o.name: [list(r) for r in o.matrix_world] for o in bpy.data.objects}, open(os.path.join(W, "tf_src.json"), "w"))
json.dump(sorted(m.name for m in bpy.data.materials), open(os.path.join(W, "cad_mats.json"), "w"))
dist = getattr(cfg, "PRE_WELD_M", 1e-6)
n = 0
for me in bpy.data.meshes:
    if me.users:
        n += build.weld(me, dist) > 0
print("PREP", len(bpy.data.objects), "objects;", n, "meshes welded at", dist, flush=True)
# explode.py reads its outputs from sys.argv after "--"
sys.argv = sys.argv[:sys.argv.index("--") + 1] + [os.path.join(W, "exploded.blend"), os.path.join(W, "explode.json")]
import explode  # noqa: E402
explode.main()
