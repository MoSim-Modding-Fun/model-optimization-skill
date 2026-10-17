# One MK4n module from the vendor CAD into its own work file, in the MK4i asset's frame: L-plate outer corner at the
# origin, base-plate bottom at z = 0, module reaching into -x / +y. Parts keep their names (pipeline input).
#   blender -b mk4ns_cad.blend --python mk4n_extract.py -- "SDS MK4n <1>" A out.blend
import bpy, sys, math, json, numpy as np
from mathutils import Matrix
root_name, tag, out = sys.argv[sys.argv.index("--")+1:][:3]
R = bpy.data.objects[root_name]
meshes = [o for o in R.children_recursive if o.type == 'MESH' and len(o.data.polygons)]
lp = max((o for o in meshes if o.name.startswith("MK4i Master Part V2")), key=lambda o: o.dimensions.length)
co = np.array([lp.matrix_world @ v.co for v in lp.data.vertices])
lo, hi = co.min(0), co.max(0)
corners = [(x, y) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])]
near = [np.min(np.hypot(co[:, 0]-x, co[:, 1]-y)) for x, y in corners]
empty = int(np.argmax(near)); outer = corners[3 - empty]          # diagonal of the empty (inner) corner
inner = corners[empty]
d = np.array(inner) - np.array(outer)                             # points from outer corner into the module
# rotate about z so that d points to (-x, +y)
ang = math.atan2(d[1], d[0]); target = math.atan2(1, -1)
k = round((target - ang) / (math.pi / 2)); rot = Matrix.Rotation(k * math.pi / 2, 4, 'Z')
T = rot @ Matrix.Translation((-outer[0], -outer[1], 0))
print("EXTRACT L plate", lp.name, "outer corner mm", np.round(np.array(outer)*1000, 1).tolist(), "rot deg", k*90)
keep = set(o.name for o in meshes)
W = {o.name: T @ o.matrix_world for o in meshes}
for o in list(bpy.data.objects):
    if o.name not in keep:
        bpy.data.objects.remove(o)
root = bpy.data.objects.new(f"MK4N {tag}", None); bpy.context.scene.collection.objects.link(root)
for o in meshes:
    o.parent = None; o.matrix_world = W[o.name]; o.parent = root; o.matrix_parent_inverse.identity(); o.matrix_world = W[o.name]
    for c in o.users_collection:
        if c != bpy.context.scene.collection: c.objects.unlink(o)
    if o.name not in bpy.context.scene.collection.objects: bpy.context.scene.collection.objects.link(o)
for c in list(bpy.data.collections): bpy.data.collections.remove(c)
bpy.ops.outliner.orphans_purge(do_recursive=True)
allco = np.array([o.matrix_world @ v.co for o in meshes for v in o.data.vertices])
print("EXTRACT", tag, "parts", len(meshes), "bbox mm", np.round(allco.min(0)*1000).tolist(), np.round(allco.max(0)*1000).tolist())
json.dump({"T": [list(r) for r in T], "root": root_name}, open(out.replace(".blend", "_frame.json"), "w"))
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=out)
