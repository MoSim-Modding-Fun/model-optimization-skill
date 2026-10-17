# Put the library "MK5n A" / "MK5n B" assets onto a robot's SDS MK5n modules.
#   1. frames, from the robot CAD (exact, same rule as mk5n_extract.py):
#        blender -b robot_cad.blend --python mk5n_place.py -- frames <frames.json> "SDS MK5n <1>=A" "SDS MK5n <2>=A" ...
#   2. placement, on the simplified robot:
#        blender -b simplified.blend --python mk5n_place.py -- place <frames.json> <out.blend>
# Each module empty gets one child "<empty> [MK5n A]" (the asset joined into one mesh, shared by modules of a type,
# live Smooth by Angle + WeightedNormal) at the module's frame; the module's old pieces (everything under the empty but
# the motors) go to "REVIEW - Replaced by library". Motors stay as their own library objects (owner: no motors in module
# assets). Check printed per module: asset vertices vs the old pieces' surfaces (median / p90 mm).
# Reverse check (one CAD has the corner block inside the module assembly, another in the drivetrain, and the asset was
# built from the latter): an old piece whose surface is mostly far from the asset AND exposed (rays from it escape
# the asset in several of the 26 grid directions, so it is not one of the hidden interior pieces the asset pruned) is not
# in the asset: it stays on the robot as it is ("MK5N keep"). MK5N_REPLACE_ALL=1 turns this off.
import bpy, sys, os, json, itertools
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import align, libcombo, shading
a = sys.argv[sys.argv.index("--") + 1:]
# MK5N_LIBFILE: another library file with the same layout (e.g. "MK5n Swerve Module (Lifted Turret)", types
# "A Lifted" / "B Lifted" in the frames)
LIBFILE = os.environ.get("MK5N_LIBFILE", f"{align.LIB}/MK5n Swerve Module/MK5n Swerve Module.blend")
MOTOR = ("Kraken", "kraken cap")


def module_meshes(empty):
    return [o for o in empty.children_recursive if o.type == 'MESH' and len(o.data.polygons)]


FAR_MM, MISSING_FRAC = 2.0, 0.25
DIRS = [Vector(d).normalized() for d in itertools.product((-1, 0, 1), repeat=3) if any(d)]


def not_in_asset(o, abvh, dg, module):
    """Area fraction of the piece's faces that are > FAR_MM from the placed asset and exposed (rays escape the asset in
    3+ of the 26 grid directions). Over MISSING_FRAC: the asset doesn't have this piece."""
    ev = o.evaluated_get(dg); m = ev.to_mesh()
    mw = o.matrix_world
    tot = miss = 0.0
    polys = list(m.polygons)
    step = max(1, len(polys) // 600)
    for p_ in polys[::step]:
        c = mw @ p_.center
        a_ = p_.area
        tot += a_
        if abvh.find_nearest(c)[3] * 1000 <= FAR_MM:
            continue
        esc = sum(abvh.ray_cast(c + d * 1e-4, d)[0] is None for d in DIRS)
        if esc >= 3:
            miss += a_
    ev.to_mesh_clear()
    frac = miss / tot if tot else 0.0
    if frac > MISSING_FRAC:
        print("MK5N keep", o.name, "under", module, "- not in the asset: %.0f%% of its area is > %.0f mm from it and exposed"
              % (100 * frac, FAR_MM), flush=True)
        return True
    if frac > 0.05:
        print("MK5N note", o.name, "%.0f%% far + exposed (replaced)" % (100 * frac), flush=True)
    return False


if a[0] == "frames":
    import mk5n_extract
    out, specs = a[1], dict(s.split("=") for s in a[2:])
    frames = {}
    ref = {}
    for name, tag in specs.items():
        E = bpy.data.objects[name]
        ms = module_meshes(E)
        T, outer, rot, z0 = mk5n_extract.module_frame(ms)
        frames[name] = {"type": tag, "T": [list(r) for r in T], "rot_deg": rot}
        # modules of one type must coincide piece by piece in the asset frame (instanced CAD)
        # (several pieces share a mesh, e.g. the five module bolts: match each to its nearest same-mesh piece)
        key = [(o.data.name, np.array(T @ o.matrix_world)) for o in ms if not o.name.startswith(MOTOR)]
        if tag not in ref:
            ref[tag] = key
        else:
            dev = max(min(np.abs(m - m2).max() for n2, m2 in ref[tag] if n2 == n) for n, m in key)
            print("MK5N frame", name, tag, "rot", rot, "max matrix deviation from the first", tag, "module:", dev, flush=True)
    json.dump(frames, open(out, "w"), indent=1)
    print("MK5N frames saved", out, flush=True)

elif a[0] == "place":
    frames, out = json.load(open(a[1])), a[2]
    sc = bpy.context.scene
    rev = bpy.data.collections.get("REVIEW - Replaced by library") or bpy.data.collections.new("REVIEW - Replaced by library")
    if rev.name not in sc.collection.children:
        sc.collection.children.link(rev)
    rev.color_tag = 'COLOR_04'
    meshes = {}
    for name, f in frames.items():
        tag = f["type"]
        if tag not in meshes:
            meshes[tag] = libcombo.collection_mesh(LIBFILE, f"MK5n {tag}")
            meshes[tag].name = f"MK5n {tag} [library]"
        E = bpy.data.objects[name]
        old = [o for o in module_meshes(E) if not o.name.startswith(MOTOR)
               and not any(c.name.startswith("REVIEW") for c in o.users_collection)]
        M = Matrix(f["T"]).inverted()
        ob = bpy.data.objects.new(f"{name} [MK5n {tag}]", meshes[tag])
        for c in E.users_collection:
            c.objects.link(ob)
        ob.parent = E
        ob.matrix_parent_inverse.identity()
        ob.matrix_world = M
        shading.add_shading(ob)
        # check against the pieces it replaces (evaluated, world space)
        dg = bpy.context.evaluated_depsgraph_get()
        V, F = [], []
        for o in old:
            ev = o.evaluated_get(dg); m = ev.to_mesh()
            base = len(V)
            V += [tuple(o.matrix_world @ v.co) for v in m.vertices]
            F += [tuple(base + i for i in p.vertices) for p in m.polygons]
            ev.to_mesh_clear()
        bvh = BVHTree.FromPolygons(V, F)
        pts = [M @ v.co for v in meshes[tag].vertices][::3]
        dist = np.array([bvh.find_nearest(p)[3] for p in pts]) * 1000
        if os.environ.get("MK5N_REPLACE_ALL") != "1":
            abvh = BVHTree.FromPolygons([tuple(M @ v.co) for v in meshes[tag].vertices],
                                        [tuple(p_.vertices) for p_ in meshes[tag].polygons])
            kept = [o for o in old if not_in_asset(o, abvh, dg, name)]
            old = [o for o in old if o not in kept]
        print("MK5N place", ob.name, "replaces", len(old), "pieces; asset verts vs old pieces: median %.3f mm, p90 %.3f mm"
              % (np.median(dist), np.percentile(dist, 90)), flush=True)
        for o in old:
            for c in list(o.users_collection):
                c.objects.unlink(o)
            rev.objects.link(o)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=out, compress=True)
    print("MK5N saved", out, flush=True)
