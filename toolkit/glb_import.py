# Import a .glb / .gltf into an empty scene with the default glTF importer, save compressed, print what the next steps
# need (objects, empties, roots, meshes, materials, tris, world bbox: the up axis and the floor show in it).
#   MEM=10G bl.sh --factory-startup --python glb_import.py -- <in.glb> <out.blend>
# Memory and time scale with the glTF's primitive count more than its size: Onshape writes one primitive per B-rep face,
# and a 680 MB glb (8M tris, 271k primitives) took 12 min at ~4.2 GB resident; meshes with tens of thousands of
# primitives take minutes each. Read the glb's JSON chunk first (node/mesh/accessor counts) to plan MEM and the wait.
import bpy, sys, time
from mathutils import Vector
a = sys.argv[sys.argv.index("--") + 1:]
t = time.time()
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.import_scene.gltf(filepath=a[0])
print("IMPORT s", round(time.time() - t, 1), flush=True)
objs = list(bpy.context.scene.objects)
mesh = [o for o in objs if o.type == 'MESH']
print("objects", len(objs), "mesh", len(mesh), "empties", sum(o.type == 'EMPTY' for o in objs),
      "meshes", len(bpy.data.meshes), "mats", len(bpy.data.materials), "roots", sum(o.parent is None for o in objs), flush=True)
lo = Vector((1e9,) * 3); hi = Vector((-1e9,) * 3)
for o in mesh:
    for c in o.bound_box:
        w = o.matrix_world @ Vector(c)
        lo = Vector(map(min, lo, w)); hi = Vector(map(max, hi, w))
print("bbox mm", [round(x * 1000) for x in lo], [round(x * 1000) for x in hi], "tris",
      sum(sum(len(p.vertices) - 2 for p in o.data.polygons) for o in mesh), flush=True)
bpy.ops.wm.save_as_mainfile(filepath=a[1], compress=True)
print("SAVED s", round(time.time() - t, 1), flush=True)
