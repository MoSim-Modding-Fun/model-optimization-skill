# Library fits for CAD meshes that merge several catalogue parts (gearbox + motor, swerve module + motors).
#  find_embedded(): locate an exact copy of a known CAD part (e.g. the robot's standalone Kraken) inside a bigger
#                   CAD mesh with one-sided ICP (part -> host) from axis-aligned starts over a grid of offsets.
#  remove_near():   host triangles not covered by the embedded copies (what is left for the next fit).
import itertools
import numpy as np
from mathutils import kdtree, Vector
import geo, align


def _tree(P):
    t = kdtree.KDTree(len(P))
    for i, p in enumerate(P):
        t.insert(p, i)
    t.balance()
    return t


def _nn(tree, X):
    d = np.empty(len(X))
    idx = np.empty(len(X), np.int64)
    for k, x in enumerate(X):
        _, i, dist = tree.find(x)
        d[k], idx[k] = dist, i
    return d, idx


def _icp(S, T, tree, R, t, iters, trim=90):
    for _ in range(iters):
        X = S @ R.T + t
        d, i = _nn(tree, X)
        keep = d <= np.percentile(d, trim)
        dR, dt = align._kabsch(X[keep], T[i[keep]])
        R, t = dR @ R, dR @ t + dt
    return R, t


def find_embedded(part_co, part_tri, host_co, host_tri, grid=3, rng=None):
    """Best rigid (R, t) mapping the part into the host, and the mean one-sided distance part->host (metres)."""
    rng = rng or np.random.default_rng(2)
    S, _ = geo.sample_surface(part_co, part_tri, 400, rng)
    T, _ = geo.sample_surface(host_co, host_tri, 12000, rng)
    tree = _tree(T)
    cS = S.mean(0)
    lo, hi = host_co.min(0), host_co.max(0)
    ext = np.ptp(part_co, axis=0)
    # candidate centres: a grid over the host box, shrunk by half the part's smallest extent
    m = 0.5 * ext.min()
    axes = [np.linspace(lo[k] + m, hi[k] - m, grid) if hi[k] - lo[k] > 2 * m else np.array([(lo[k] + hi[k]) / 2]) for k in range(3)]
    starts = []
    for R0 in align._rotations24():
        for c in itertools.product(*axes):
            t0 = np.array(c) - R0 @ cS
            R1, t1 = _icp(S, T, tree, R0, t0, 6)
            d, _ = _nn(tree, S @ R1.T + t1)
            starts.append((float(d.mean()), R1, t1))
    starts.sort(key=lambda x: x[0])
    S2, _ = geo.sample_surface(part_co, part_tri, 2500, rng)
    T2, _ = geo.sample_surface(host_co, host_tri, 60000, rng)
    tree2 = _tree(T2)
    best = None
    for _, R, t in starts[:6]:
        R, t = _icp(S2, T2, tree2, R, t, 25, trim=95)
        d, _ = _nn(tree2, S2 @ R.T + t)
        if best is None or d.mean() < best[2]:
            best = (R, t, float(d.mean()), float(np.percentile(d, 95)))
    return best


def covered_faces(host_co, host_tri, parts, tol=1.0e-3):
    """Boolean per host triangle: all three corners lie within tol of one of the placed parts' surfaces."""
    cov = np.zeros(len(host_tri), bool)
    for co, tri in parts:
        P, _ = geo.sample_surface(co, tri, 80000, np.random.default_rng(3))
        tree = _tree(P)
        dv, _ = _nn(tree, host_co)
        near = dv <= tol
        cov |= near[host_tri].all(1)
    return cov


# ---------------------------------------------------------------- joining library meshes

def join_meshes(name, parts):
    """One mesh from [(mesh, 4x4 matrix)], materials merged by datablock (first use wins the slot)."""
    import bpy, bmesh
    bm = bmesh.new()
    mats = []
    for me, M in parts:
        tmp = me.copy()
        tmp.transform(M)
        remap = []
        for m in tmp.materials:
            if m not in mats:
                mats.append(m)
            remap.append(mats.index(m))
        n0 = len(bm.faces)
        bm.from_mesh(tmp)
        bm.faces.ensure_lookup_table()
        for f in bm.faces[n0:]:
            f.material_index = remap[f.material_index] if remap else 0
        bpy.data.meshes.remove(tmp)
    out = bpy.data.meshes.new(name)
    bm.to_mesh(out)
    bm.free()
    for m in mats:
        out.materials.append(m)
    return out


def collection_mesh(blend, coll_name):
    """A library collection asset as ONE mesh in the collection's frame (all objects evaluated; the Smooth by Angle /
    WeightedNormal stack is left off: the receiving object gets its own live shading modifiers)."""
    import bpy, shading
    with bpy.data.libraries.load(blend, link=False) as (src, dst):
        dst.collections = [coll_name]
    c = dst.collections[0]
    bpy.context.scene.collection.children.link(c)
    for o in c.all_objects:
        for m in o.modifiers:
            if m.type == 'WEIGHTED_NORMAL' or (m.type == 'NODES' and m.node_group and m.node_group.name.startswith(shading.SBA)):
                m.show_viewport = m.show_render = False
    dg = bpy.context.evaluated_depsgraph_get()
    parts = []
    for o in c.all_objects:
        if o.type == 'MESH':
            parts.append((bpy.data.meshes.new_from_object(o.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg), o.matrix_world.copy()))
    out = join_meshes(coll_name + " [library]", parts)
    for me, _ in parts:
        bpy.data.meshes.remove(me)
    for o in list(c.all_objects):
        bpy.data.objects.remove(o)
    bpy.data.collections.remove(c)
    return out


def fit_combo(me, libs, max_mm=3.0):
    """CAD mesh = gearbox + motor merged. libs = [gearbox, motor] library meshes. The motor asset is fitted to the
    best-matching CAD piece among the largest ones, the gearbox asset to everything else. Returns (mesh, info)."""
    import explode
    co, tri, _ = geo.mesh_arrays(me)
    tp = np.empty(len(me.loop_triangles), np.int64)
    me.loop_triangles.foreach_get("polygon_index", tp)
    pl = np.empty(len(me.polygons), np.int64)
    isl = explode.islands(me)
    for k, f in enumerate(isl):
        pl[f] = k
    ti = pl[tp]
    big = np.argsort([-len(f) for f in isl])[:6]
    gbx, mot = libs
    mco, mtri, _ = geo.mesh_arrays(mot)
    best = None
    for k in big:
        sel = ti == k
        if sel.sum() < 50:
            continue
        R, t, ch = align.align(mco, mtri, co, tri[sel])
        if best is None or sum(ch) < sum(best[3]):
            best = (k, R, t, ch)
    info = {"motor_chamfer_mm": round(sum(best[3]) / 2 * 1000, 2)}
    if sum(best[3]) / 2 > max_mm * 1e-3:
        return None, info
    gco, gtri, _ = geo.mesh_arrays(gbx)
    R2, t2, ch2 = align.align(gco, gtri, co, tri[ti != best[0]])
    info["gearbox_chamfer_mm"] = round(sum(ch2) / 2 * 1000, 2)
    if sum(ch2) / 2 > max_mm * 1e-3:
        return None, info
    out = join_meshes("combo", [(gbx, align.to_matrix(R2, t2)), (mot, align.to_matrix(best[1], best[2]))])
    info["motor_matrix"] = align.to_matrix(best[1], best[2])
    return out, info
