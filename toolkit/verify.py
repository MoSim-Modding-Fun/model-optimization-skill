import bpy, sys, json, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robots
cfg = robots.load()
args = sys.argv[sys.argv.index("--")+1:]
orig_names = set(json.load(open(args[0])))
out_fbx = args[1]
objs = bpy.data.objects
names = set(o.name for o in objs)
missing = sorted(orig_names - names)
added = sorted(names - orig_names)
print("VERIFY objects", len(objs), "missing", len(missing), missing[:10], "added", added)
orphan_parent = [o.name for o in objs if o.parent is None and o.name != cfg.ROOT]
print("VERIFY unparented roots", orphan_parent[:10])
print("VERIFY collections", [(c.name, len(c.objects)) for c in bpy.data.collections])
bpy.ops.export_scene.fbx(filepath=out_fbx, use_selection=False, apply_scale_options='FBX_SCALE_ALL', use_mesh_modifiers=True, path_mode='AUTO')
print("VERIFY fbx", os.path.getsize(out_fbx))
