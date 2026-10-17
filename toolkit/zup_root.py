# Z-up work copy for an export that stands on +Y (wheels at y = 0) and has no root object to turn.
#   bl.sh <in.blend> --python zup_root.py -- make <out.blend>       every unparented object goes under a new empty
#       "_zup_root" (world matrices kept) and ONLY that empty turns +90 deg about X (+Y -> +Z); the pipeline needs Z up.
#   bl.sh <run>/final.blend --python zup_root.py -- restore <out.blend>   the deliverable: root rotation -> identity,
#       every child unparented with its world matrix (= the source's), root removed. Objects the run added must sit
#       under the root (or anywhere below it) so they come back too. Then tfcheck.py against the ORIGINAL import (0.0).
# input-types.md §6 has the rotated-root recipe for exports that do have a root; this is the rootless variant.
import bpy, sys, math
from mathutils import Vector
a = sys.argv[sys.argv.index("--") + 1:]
mode, OUT = a[0], a[1]
sc = bpy.context.scene
bpy.context.preferences.filepaths.save_version = 0
if mode == "make":
    root = bpy.data.objects.new("_zup_root", None)
    sc.collection.objects.link(root)
    for o in [o for o in sc.objects if o.parent is None and o is not root]:
        mw = o.matrix_world.copy()
        o.parent = root
        o.matrix_parent_inverse.identity()
        o.matrix_basis = mw
    root.rotation_mode = 'XYZ'
    root.rotation_euler = (math.pi / 2, 0.0, 0.0)
elif mode == "restore":
    root = bpy.data.objects["_zup_root"]
    root.rotation_mode = 'XYZ'
    root.rotation_euler = (0.0, 0.0, 0.0); root.location = (0, 0, 0); root.scale = (1, 1, 1)
    bpy.context.view_layer.update()
    for c in list(root.children):
        mw = c.matrix_world.copy()
        c.parent = None
        c.matrix_world = mw
    bpy.data.objects.remove(root)
bpy.context.view_layer.update()
lo, hi = [1e9] * 3, [-1e9] * 3
for o in sc.objects:
    if o.type == 'MESH':
        for c in o.bound_box:
            w = o.matrix_world @ Vector(c)
            lo = [min(x, y) for x, y in zip(lo, w)]; hi = [max(x, y) for x, y in zip(hi, w)]
print("ZUP", mode, "bbox mm", [round(x * 1000) for x in lo], [round(x * 1000) for x in hi], flush=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
