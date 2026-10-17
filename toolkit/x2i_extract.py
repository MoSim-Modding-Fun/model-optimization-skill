# One WCP Swerve X2i module from a robot CAD into its own work file, in the MK4i asset's frame (2026-10-15):
# outer corner of the tube-mount bottom plate (WCP-1705) at the origin, that plate's bottom (= frame-rail top) at z = 0,
# module reaching into -x / +y. Parts keep their names (pipeline input, ROBOT=x2i). Also writes <out>_frame.json with
#   T      world -> asset frame for this module
#   key    the key piece (largest WCP-1702 base plate) and its world matrix, so other modules' frames follow from theirs
#   turret pieces whose transform relative to the key piece differs between the robot's modules (= the steering turret)
#   blender -b robot.blend --python x2i_extract.py -- "(Low) WCP Swerve X2i <1>" out.blend
import bpy, sys, math, json, re
import numpy as np
from mathutils import Matrix
root_name, out = sys.argv[sys.argv.index("--") + 1:][:2]
MOD_RX = re.compile(r"^\(Low\) WCP Swerve X2i <\d>$")


def meshes_of(e):
    return [o for o in e.children_recursive if o.type == 'MESH' and len(o.data.polygons)]


def key_piece(ms):
    base = [o for o in ms if o.name.startswith("WCP Swerve X2i Base Kit")]
    return max(base, key=lambda o: o.dimensions[0] * o.dimensions[1] * o.dimensions[2])


def module_frame(ms):
    """World -> asset matrix: tube-mount bottom plate's outer corner at the origin, its bottom at z = 0, module in -x/+y."""
    bp = next(o for o in ms if "Tube Mount Bottom Plate" in o.name)
    co = np.array([bp.matrix_world @ v.co for v in bp.data.vertices])
    lo, hi = co.min(0), co.max(0)
    corners = [(x, y) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])]
    outer = max(corners, key=lambda c: math.hypot(*c))           # farthest from the robot centre
    inner = min(corners, key=lambda c: math.hypot(*c))
    d = np.array(inner) - np.array(outer)
    ang = math.atan2(d[1], d[0]); target = math.atan2(1, -1)
    k = round((target - ang) / (math.pi / 2))
    T = Matrix.Rotation(k * math.pi / 2, 4, 'Z') @ Matrix.Translation((-outer[0], -outer[1], -lo[2]))
    return T, outer, (k * 90) % 360, lo[2]


E = bpy.data.objects[root_name]
ms = meshes_of(E)
T, outer, rot, z0 = module_frame(ms)
K = key_piece(ms)
# turret = pieces whose pose relative to the key piece differs in another module of the robot
others = [o for o in bpy.data.objects if o.type == 'EMPTY' and MOD_RX.match(o.name) and o.name != root_name]
rel = lambda o, k: np.array(k.matrix_world.inverted() @ o.matrix_world)
turret = set()
for e2 in others:
    ms2 = meshes_of(e2); k2 = key_piece(ms2)
    by_mesh = {}
    for o2 in ms2:
        by_mesh.setdefault(o2.data.name, []).append(rel(o2, k2))
    for o in ms:
        r = rel(o, K)
        if min((np.abs(r - r2).max() for r2 in by_mesh.get(o.data.name, [])), default=1) > 1e-4:
            turret.add(o.name)
print("X2I extract", root_name, "pieces", len(ms), "turret", len(turret), "outer mm", np.round(np.array(outer) * 1000, 1).tolist(),
      "rot", rot, "z0 mm", round(z0 * 1000, 2), flush=True)
K_world = [list(r) for r in K.matrix_world]   # before the module moves into the asset frame
W = {o.name: T @ o.matrix_world for o in ms}
keep = {o.name for o in ms}
for o in list(bpy.data.objects):
    if o.name not in keep:
        bpy.data.objects.remove(o)
root = bpy.data.objects.new("X2I", None)
bpy.context.scene.collection.objects.link(root)
for o in ms:
    o.parent = None; o.matrix_world = W[o.name]; o.parent = root; o.matrix_parent_inverse.identity(); o.matrix_world = W[o.name]
    for c in o.users_collection:
        if c != bpy.context.scene.collection:
            c.objects.unlink(o)
    if o.name not in bpy.context.scene.collection.objects:
        bpy.context.scene.collection.objects.link(o)
for c in list(bpy.data.collections):
    bpy.data.collections.remove(c)
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
allco = np.array([o.matrix_world @ v.co for o in ms for v in o.data.vertices])
print("X2I bbox mm", np.round(allco.min(0) * 1000).tolist(), np.round(allco.max(0) * 1000).tolist(), flush=True)
json.dump({"T": [list(r) for r in T], "root": root_name, "rot_deg": rot, "key": K.name,
           "key_world": K_world, "turret": sorted(turret)},
          open(out.replace(".blend", "_frame.json"), "w"), indent=1)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=out, compress=True)
