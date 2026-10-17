# One assembly or one part of a robot CAD into its own work file, in its own frame: the start of a library
# asset build (kit_asset.py). An EMPTY keeps its subtree and is moved to the world origin (so the pieces sit in the
# assembly's frame, the frame LIB_EMPTY_IDENTITY places the asset in); a MESH object is put at identity (its mesh is
# in the vendor part frame, the frame LIB_IDENTITY swaps it in). Everything else is deleted. Read-only on the source.
#   blender -b robot.blend --python kit_extract.py -- out_work.blend "<empty or object name>" ["<key piece regex>"]
# Onshape glTF assembly empties sit at the world origin with their pieces baked in world space, so an assembly has no
# frame of its own: with a key piece regex the work file is put in that piece's frame (its vendor part frame: a motor
# body, a wheel hub), the frame pipeline.py's LIB_EMPTY_IDENTITY places the asset in for every instance.
import bpy, sys, re
from mathutils import Matrix

OUT, NAME = sys.argv[sys.argv.index("--") + 1:][:2]
KEY = sys.argv[sys.argv.index("--") + 3] if len(sys.argv) > sys.argv.index("--") + 3 else None
root = bpy.data.objects[NAME]
keep = {root} | set(root.children_recursive)
for o in list(bpy.data.objects):
    if o not in keep:
        bpy.data.objects.remove(o)
mw = {o.name: o.matrix_world.copy() for o in keep}
inv = mw[root.name].inverted()
root.parent = None
root.matrix_world = Matrix.Identity(4)
bpy.context.view_layer.update()
if KEY:
    keyo = sorted([o for o in keep if o.type == 'MESH' and re.search(KEY, o.name)], key=lambda o: o.name)[0]
    K = (inv @ mw[keyo.name]).inverted()
    root.matrix_world = K
    inv = K @ inv
    bpy.context.view_layer.update()
    print("KEY", keyo.name, flush=True)
# every piece must keep its pose relative to the root
err = max((max(abs(a - b) for ra, rb in zip(inv @ mw[o.name], o.matrix_world) for a, b in zip(ra, rb)) for o in keep),
          default=0.0)
for c in list(bpy.data.collections):
    if not c.all_objects:
        bpy.data.collections.remove(c)
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
tris = sum(sum(len(p.vertices) - 2 for p in o.data.polygons) for o in keep if o.type == 'MESH')
print("EXTRACT", NAME, "objects", len(keep), "meshes", len({o.data.name for o in keep if o.type == 'MESH'}), "tris", tris,
      "frame check", round(err, 9), flush=True)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
