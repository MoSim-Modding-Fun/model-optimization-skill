# Put the library "WCP Swerve X2i" asset onto a robot's X2i modules (after the pipeline; like mk5n_place.py).
#   blender -b simplified.blend --python x2i_place.py -- <x2i_work_frame.json> <out.blend> [library .blend] [asset name]
# The frame json comes from x2i_extract.py on the robot CAD: T (world -> asset frame) for the extracted module and its
# key piece (the largest WCP-1702 base plate). Every module empty "(Low) WCP Swerve X2i <n>" gets one child
# "<empty> [WCP Swerve X2i]" (the asset joined into one mesh, shared, live Smooth by Angle + WeightedNormal) at
#   asset world = K_n @ K_ref^-1 @ T^-1      (K = the key piece's world matrix; the same mesh in every module)
# so modules turned on the robot follow their own base plate. The steering turret comes from the extracted module (owner:
# wheel angle never matters). The module's old pieces (everything under the empty but the motors) go to
# "REVIEW - Replaced by library"; an old piece the asset doesn't have (mostly > 2 mm from it and exposed) stays
# ("X2I keep", as mk5n_place). Check printed per module: asset vertices vs the old pieces' surfaces (median / p90 mm).
import bpy, sys, os, json, re, itertools
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import align, libcombo, shading
a = sys.argv[sys.argv.index("--") + 1:]
FRAME, OUT = a[:2]
LIBFILE = a[2] if len(a) > 2 else f"{align.LIB}/WCP Swerve X2i/WCP Swerve X2i.blend"
NAME = a[3] if len(a) > 3 else "WCP Swerve X2i"
MOD_RX = re.compile(r"^\(Low\) WCP Swerve X2i <\d>$")
MOTOR = ("Kraken", "kraken cap")
FAR_MM, MISSING_FRAC = 2.0, 0.25
DIRS = [Vector(d).normalized() for d in itertools.product((-1, 0, 1), repeat=3) if any(d)]


def module_meshes(e):
    return [o for o in e.children_recursive if o.type == 'MESH' and len(o.data.polygons)]


def key_piece(ms):
    base = [o for o in ms if o.name.startswith("WCP Swerve X2i Base Kit")]
    return max(base, key=lambda o: o.dimensions[0] * o.dimensions[1] * o.dimensions[2])


def not_in_asset(o, abvh, dg, module):
    ev = o.evaluated_get(dg); m = ev.to_mesh()
    mw = o.matrix_world
    tot = miss = 0.0
    polys = list(m.polygons)
    for p_ in polys[::max(1, len(polys) // 600)]:
        c = mw @ p_.center
        tot += p_.area
        if abvh.find_nearest(c)[3] * 1000 <= FAR_MM:
            continue
        if sum(abvh.ray_cast(c + d * 1e-4, d)[0] is None for d in DIRS) >= 3:
            miss += p_.area
    ev.to_mesh_clear()
    frac = miss / tot if tot else 0.0
    if frac > MISSING_FRAC:
        print("X2I keep", o.name, "under", module, "- not in the asset: %.0f%% of its area is > %.0f mm from it and exposed"
              % (100 * frac, FAR_MM), flush=True)
        return True
    return False


f = json.load(open(FRAME))
T = Matrix(f["T"]); Kref = Matrix(f["key_world"])
sc = bpy.context.scene
rev = bpy.data.collections.get("REVIEW - Replaced by library") or bpy.data.collections.new("REVIEW - Replaced by library")
if rev.name not in sc.collection.children:
    sc.collection.children.link(rev)
rev.color_tag = 'COLOR_04'
# turret pieces (steered: a module's own steer angle differs from the asset's) are always replaced, never "kept"
TURRET_MESHES = {bpy.data.objects[n].data.name for n in f.get("turret", []) if n in bpy.data.objects}
mesh = libcombo.collection_mesh(LIBFILE, NAME)
mesh.name = f"{NAME} [library]"
for E in sorted([o for o in bpy.data.objects if o.type == 'EMPTY' and MOD_RX.match(o.name)], key=lambda o: o.name):
    ms = module_meshes(E)
    K = key_piece(ms)
    M = K.matrix_world @ Kref.inverted() @ T.inverted()
    old = [o for o in ms if not o.name.startswith(MOTOR) and not any(c.name.startswith("REVIEW") for c in o.users_collection)]
    ob = bpy.data.objects.new(f"{E.name} [{NAME}]", mesh)
    for c in K.users_collection:
        c.objects.link(ob)
    ob.parent = E
    ob.matrix_parent_inverse.identity()
    ob.matrix_world = M
    shading.add_shading(ob)
    dg = bpy.context.evaluated_depsgraph_get()
    V, F = [], []
    for o in old:
        ev = o.evaluated_get(dg); m = ev.to_mesh()
        base = len(V)
        V += [tuple(o.matrix_world @ v.co) for v in m.vertices]
        F += [tuple(base + i for i in p.vertices) for p in m.polygons]
        ev.to_mesh_clear()
    bvh = BVHTree.FromPolygons(V, F)
    pts = [M @ v.co for v in mesh.vertices][::3]
    dist = np.array([bvh.find_nearest(p)[3] for p in pts]) * 1000
    abvh = BVHTree.FromPolygons([tuple(M @ v.co) for v in mesh.vertices], [tuple(p_.vertices) for p_ in mesh.polygons])
    kept = [o for o in old if o.data.name not in TURRET_MESHES and not_in_asset(o, abvh, dg, E.name)]
    old = [o for o in old if o not in kept]
    print("X2I place", ob.name, "replaces", len(old), "pieces, keeps", len(kept), "; asset verts vs old pieces: median %.3f mm,"
          " p90 %.3f mm" % (np.median(dist), np.percentile(dist, 90)), flush=True)
    for o in old:
        for c in list(o.users_collection):
            c.objects.unlink(o)
        rev.objects.link(o)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
print("X2I saved", OUT, flush=True)
