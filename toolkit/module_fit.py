# Library swerve-module assets onto a robot whose modules have NO assembly empties (e.g. an export where every copy of a
# part is merged and exploded into loose pieces): each module = the pieces cfg.group_of puts in one corner group.
#   ROBOT=<robot> bl.sh exploded.blend --python module_fit.py -- fit <frames.json>
#   ROBOT=<robot> bl.sh <run>/simplified_exploded.blend --python module_fit.py -- place <frames.json> <out.blend> <report.json>
# Config (robots/r<name>.py):
#   MODULE_FIT = {"lib": "MK4n Swerve Module/MK4n Swerve Module.blend",       # relative to Components/
#                 "types": {"A": "MK4n A", "B": "MK4n B"},                     # tag -> collection asset (mirror types)
#                 "corners": {"swerve fl": (-0.318, 0.368), ...},             # group -> frame corner xy, m, work frame
#                 "frame_top_z": 0.095,                                         # frame-rail top, m (asset z = 0 there)
#                 "vendor": r"^(MK4|MK3|Colson Performa|Wheel Mount|Bevel Gear|...)",  # parts always replaced
#                 "skip_target": ("Through Bore Encoder", "SHCS", ...),       # parts left out of the fit target
#                 "skip": ("<motor part>", ...)}                                # never touched (library motors)
# fit: per corner, the asset's fixed parts (everything not under its "<type> Wheel" empty: steer angle never matters) are
#   fitted to the module's CAD pieces by trimmed ICP from the frame corner, every type x 4 quarter turns; best by median
#   asset-to-CAD distance (well under 1 mm = the module matches). Writes frames + per-module stats.
# place (post step before implode): one object "<group> [<collection>]" under the work root (or the scene root) per
#   module, one shared mesh per type, live Smooth by Angle -> WeightedNormal, colours voted from the module's own vendor
#   pieces (house rule: CAD colours on swerve modules). Replaced -> "REVIEW - Replaced by library": every vendor part
#   (wheel group included) and any other piece the asset covers; a piece the asset lacks (> 25 % of its area > 2 mm from
#   the asset AND exposed: rays escape the asset in 3+ of 26 directions) stays (encoders, frame plates, a gearbox next
#   to the module). Pieces already in REVIEW or holding a library mesh are skipped. Vendor pieces just outside the
#   corner group but inside the placed asset's bbox (+10 mm) are replaced too (tight group boxes missed 8 standoffs).
import bpy, sys, os, json, math, re, itertools
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robots, geo, align, paths, libcombo, shading, build  # noqa: E402
cfg = robots.load()
MF = cfg.MODULE_FIT
LIBFILE = os.path.join(paths.components(), MF["lib"])
a = sys.argv[sys.argv.index("--") + 1:]
MODE = a[0]
DIRS = [Vector(d).normalized() for d in itertools.product((-1, 0, 1), repeat=3) if any(d)]


def load_type(coll):
    """{"fixed": (co, tri), "wheel": (co, tri)} of a library collection asset in its own frame."""
    with bpy.data.libraries.load(LIBFILE, link=False) as (src, dst):
        dst.collections = [coll]
    c = dst.collections[0]
    bpy.context.scene.collection.children.link(c)
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    out = {"fixed": ([], []), "wheel": ([], [])}
    for o in [o for o in c.all_objects if o is not None and o.type == 'MESH']:
        p, wheel = o.parent, False
        while p:
            wheel |= "Wheel" in p.name
            p = p.parent
        m = bpy.data.meshes.new_from_object(o.evaluated_get(dg), depsgraph=dg)
        co, tri, _ = geo.mesh_arrays(m)
        M = np.array(o.matrix_world)
        V, F = out["wheel" if wheel else "fixed"]
        F.append(tri + sum(len(v) for v in V)); V.append(co @ M[:3, :3].T + M[:3, 3])
        bpy.data.meshes.remove(m)
    for o in [o for o in c.all_objects if o is not None]:
        bpy.data.objects.remove(o)
    bpy.data.collections.remove(c)
    return {k: (np.vstack(V), np.vstack(F)) for k, (V, F) in out.items()}


def rotz(deg):
    r = math.radians(deg)
    return np.array([[math.cos(r), -math.sin(r), 0], [math.sin(r), math.cos(r), 0], [0, 0, 1.0]])


if MODE == "fit":
    rng = np.random.default_rng(3)
    lib = {t: load_type(c) for t, c in MF["types"].items()}
    objs = [o for o in bpy.context.scene.objects if o.type == 'MESH' and len(o.data.polygons)]
    skip_t = tuple(MF.get("skip_target", ())) + tuple(MF.get("skip", ()))
    frames = {}
    for g, (cx, cy) in MF["corners"].items():
        pieces = [o for o in objs if cfg.group_of(o) == g]
        tgt = [o for o in pieces if not cfg.part(o.name).startswith(skip_t)] if skip_t else pieces
        V, F = [], []
        for o in tgt:
            co, tri = align.world_mesh_arrays(o)
            F.append(tri + sum(len(v) for v in V)); V.append(co)
        TV, TF = np.vstack(V), np.vstack(F)
        T, _ = geo.sample_surface(TV, TF, 20000, rng)
        bvh = BVHTree.FromPolygons([tuple(v) for v in TV.tolist()], TF.tolist(), all_triangles=True)
        best = None
        for tag in MF["types"]:
            S, _ = geo.sample_surface(*lib[tag]["fixed"], 4000, rng)
            for deg in (0, 90, 180, 270):
                R, t = align.icp(S, T, rotz(deg), np.array([cx, cy, MF["frame_top_z"]]), 40)
                d = np.array([bvh.find_nearest(tuple(p))[3] for p in (S @ R.T + t).tolist()]) * 1000
                med, p90 = float(np.median(d)), float(np.percentile(d, 90))
                print(f"MODULE {g} {tag} rot{deg}: median {med:.3f} mm p90 {p90:.3f} mm", flush=True)
                if best is None or med < best[0]:
                    best = (med, p90, tag, deg, R, t)
        med, p90, tag, deg, R, t = best
        frames[g] = {"type": tag, "collection": MF["types"][tag], "rot_deg": deg, "M": [list(r) for r in align.to_matrix(R, t)],
                     "median_mm": med, "p90_mm": p90, "pieces": [o.name for o in pieces]}
        print(f"MODULE BEST {g}: {MF['types'][tag]} rot{deg} median {med:.3f} p90 {p90:.3f}; {len(pieces)} pieces", flush=True)
    json.dump(frames, open(a[1], "w"), indent=1)

elif MODE == "place":
    frames, OUT, REP = json.load(open(a[1])), a[2], a[3]
    VENDOR = re.compile(MF["vendor"])
    SKIP = tuple(MF.get("skip", ()))
    sc = bpy.context.scene
    root = bpy.data.objects.get("_zup_root")
    rev = bpy.data.collections.get("REVIEW - Replaced by library") or bpy.data.collections.new("REVIEW - Replaced by library")
    if rev.name not in sc.collection.children:
        sc.collection.children.link(rev)
    dg = bpy.context.evaluated_depsgraph_get()

    def far_exposed(o, abvh):
        ev = o.evaluated_get(dg); m = ev.to_mesh()
        tot = miss = 0.0
        polys = list(m.polygons)
        for p_ in polys[::max(1, len(polys) // 400)]:
            c = o.matrix_world @ p_.center
            tot += p_.area
            if abvh.find_nearest(c)[3] * 1000 <= 2.0:
                continue
            if sum(abvh.ray_cast(c + d * 1e-4, d)[0] is None for d in DIRS) >= 3:
                miss += p_.area
        ev.to_mesh_clear()
        return (miss / tot if tot else 0.0) > 0.25

    def world_mesh(objs):
        V, F, M, mats = [], [], [], []
        for o in objs:
            ev = o.evaluated_get(dg); m = ev.to_mesh()
            base = len(V)
            V += [tuple(o.matrix_world @ v.co) for v in m.vertices]
            for p in m.polygons:
                F.append(tuple(base + i for i in p.vertices))
                mt = m.materials[p.material_index] if p.material_index < len(m.materials) else None
                if mt not in mats:
                    mats.append(mt)
                M.append(mats.index(mt))
            ev.to_mesh_clear()
        me = bpy.data.meshes.new("cad_vote"); me.from_pydata(V, [], F)
        for mt in mats:
            me.materials.append(mt)
        me.polygons.foreach_set("material_index", M); me.update()
        return me

    meshes, report = {}, {"recoloured": {}}
    for g, f in frames.items():
        coll = f["collection"]
        if coll not in meshes:
            meshes[coll] = libcombo.collection_mesh(LIBFILE, coll)
            meshes[coll].name = f"{coll} [library]"
        M = Matrix(f["M"])
        ob = bpy.data.objects.new(f"{coll} {g} [{coll}]", meshes[coll])
        sc.collection.objects.link(ob)
        if root:
            ob.parent = root; ob.matrix_parent_inverse.identity()
        ob.matrix_world = M
        bpy.context.view_layer.update()
        shading.add_shading(ob)
        awm = meshes[coll].copy(); awm.transform(M)
        abvh = BVHTree.FromPolygons([tuple(v.co) for v in awm.vertices], [tuple(p.vertices) for p in awm.polygons])
        cand = []
        for n in f["pieces"]:
            o = bpy.data.objects.get(n)
            if o is None or o.type != 'MESH' or not len(o.data.polygons):
                continue
            if any(c.name.startswith("REVIEW") for c in o.users_collection) or "[library" in o.data.name:
                continue
            if SKIP and cfg.part(o.name).startswith(SKIP):
                continue
            cand.append(o)
        # vendor pieces just outside the corner group (tight group boxes) but inside the placed asset's bbox (+10 mm)
        av = np.array([tuple(v.co) for v in awm.vertices]); lo, hi = av.min(0) - 0.01, av.max(0) + 0.01
        have = {o.name for o in cand}
        for o in sc.objects:
            if (o.type != 'MESH' or not len(o.data.polygons) or o.name in have or "[library" in o.data.name
                    or any(c.name.startswith("REVIEW") for c in o.users_collection) or not VENDOR.match(cfg.part(o.name))):
                continue
            bb = np.array([tuple(o.matrix_world @ Vector(c)) for c in o.bound_box])
            cc = 0.5 * (bb.min(0) + bb.max(0))
            if np.all(cc >= lo) and np.all(cc <= hi):
                cand.append(o); have.add(o.name)
        replaced = [o for o in cand if VENDOR.match(cfg.part(o.name)) or not far_exposed(o, abvh)]
        kept = [o for o in cand if o not in replaced]
        if coll not in report["recoloured"]:
            vote = world_mesh([o for o in replaced if VENDOR.match(cfg.part(o.name))])
            report["recoloured"][coll] = build.recolour_from_cad(vote, awm)
            for i, mt in enumerate(awm.materials):
                meshes[coll].materials[i] = mt
            bpy.data.meshes.remove(vote)
        bpy.data.meshes.remove(awm)
        for o in replaced:
            for c in list(o.users_collection):
                c.objects.unlink(o)
            rev.objects.link(o)
        report[g] = {"object": ob.name, "replaced": [o.name for o in replaced], "kept": [o.name for o in kept]}
        print(f"MODULE place {ob.name}: replaces {len(replaced)} pieces, keeps {len(kept)}: "
              f"{sorted(set(cfg.part(o.name) for o in kept))}", flush=True)
    json.dump(report, open(REP, "w"), indent=1)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
    print("MODULE saved", OUT, flush=True)
