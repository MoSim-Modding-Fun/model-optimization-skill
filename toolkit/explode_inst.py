# Instance-aware explode for exports that merged every copy of a part into ONE mesh (each object = one CAD body with
# all its copies baked in world space, no hierarchy; seen on an Onshape glTF). Every mesh object with polygons
# (cfg.EXPLODE_ALL, default True here) or only cfg.EXPLODE parts is split into its loose pieces (islands of the welded
# mesh); a piece that is a rigid copy of an earlier one (same materials, area within 1 %, PCA eigenvalues within 2 %,
# two-way vertex-to-surface distance <= INST_TOL_M after PCA-frame alignment + ICP) shares that piece's mesh and gets
# the copy's transform. Copies are tessellated separately by the exporter, so vertex order can't be used; rotationally
# symmetric pieces try every 5 deg about their axis. Piece objects are "<object>~NNN" children with custom props
# piece_of / exploded_from like explode.py, so implode.py joins them back (its cache key includes the piece matrices).
#   ROBOT=<name> bl.sh welded.blend --python explode_inst.py -- <out.blend> <explode.json>    (INST_TOL_M=5e-5 default)
# Canonical piece meshes are centred on their area centroid (the piece object's origin sits on the part). Run it on a
# welded copy (prep.py with no cfg.EXPLODE writes one). 2,727 pieces -> 2,079 meshes at 0.05 mm in ~45 s on one robot.
import bpy, sys, os, json, time, collections
import numpy as np
from mathutils import Matrix
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robots  # noqa: E402
cfg = robots.load()
import explode as X  # noqa: E402  (helpers only: islands, MeshArrays, piece_mesh)

OUT, INFO = sys.argv[sys.argv.index("--") + 1:][:2]
TOL = float(os.environ.get("INST_TOL_M", "5e-5"))
EXPLODE_ALL = getattr(cfg, "EXPLODE_ALL", True)
T0 = time.time()
sc = bpy.context.scene
bpy.context.preferences.filepaths.save_version = 0


def kabsch(Q, P):
    """Proper rotation R and t with R @ Q + t ~ P (rows are corresponding points)."""
    cq, cp = Q.mean(0), P.mean(0)
    H = (Q - cq).T @ (P - cp)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, cp - R @ cq


def tri_arrays(co, lv_new, lt):
    """Fan triangles (index triples into co), areas and centroids."""
    starts = np.concatenate(([0], np.cumsum(lt)[:-1]))
    ia, ib, ic = [], [], []
    for k in range(int(lt.max()) - 2):
        sel = lt > k + 2
        ia.append(starts[sel]); ib.append(starts[sel] + k + 1); ic.append(starts[sel] + k + 2)
    tri = np.stack([lv_new[np.concatenate(ia)], lv_new[np.concatenate(ib)], lv_new[np.concatenate(ic)]], 1)
    a, b, c = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
    ar = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    return tri, ar, (a + b + c) / 3


class Shape:
    """Rigid-invariant description of one piece: area centroid, PCA frame, eigenvalues, BVH, KD-tree."""
    def __init__(self, co, lv_new, lt):
        from mathutils.bvhtree import BVHTree
        from mathutils.kdtree import KDTree
        self.co = co
        tri, ar, cen = tri_arrays(co, lv_new, lt)
        A = max(ar.sum(), 1e-18)
        self.area = A
        self.c = (cen * ar[:, None]).sum(0) / A
        d = cen - self.c
        C = (d * ar[:, None]).T @ d / A + 1e-18 * np.eye(3)
        w, E = np.linalg.eigh(C)
        self.w, self.E = w, E
        self.bvh = BVHTree.FromPolygons([tuple(v) for v in co.tolist()], tri.tolist(), all_triangles=True)
        self.kd = KDTree(len(co))
        for i, v in enumerate(co.tolist()):
            self.kd.insert(v, i)
        self.kd.balance()
        rng = np.random.default_rng(0)
        self.sub = co if len(co) <= 400 else co[rng.choice(len(co), 400, replace=False)]

    def surf_dist(self, pts):
        out = np.empty(len(pts))
        for i, p in enumerate(pts.tolist()):
            hit = self.bvh.find_nearest(p)
            out[i] = hit[3] if hit[0] is not None else 1e9
        return out


def candidate_rotations(src, dst, rel=0.02):
    """Rotations taking src's PCA frame onto dst's: 4 proper sign flips; about a degenerate axis also every 5 deg."""
    w = dst.w
    out = []
    near = lambda a, b: abs(a - b) <= rel * max(abs(a), abs(b), 1e-12)
    deg_axis = None
    if near(w[0], w[1]) and near(w[1], w[2]):
        deg_axis = "all"
    elif near(w[0], w[1]):
        deg_axis = 2
    elif near(w[1], w[2]):
        deg_axis = 0
    for sx in (1, -1):
        for sy in (1, -1):
            S = np.diag([sx, sy, sx * sy * np.sign(np.linalg.det(src.E) * np.linalg.det(dst.E))])
            base = dst.E @ S @ src.E.T
            if np.linalg.det(base) < 0:
                continue
            if deg_axis in (0, 2):
                ax = dst.E[:, deg_axis]
                K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                for th in np.radians(np.arange(0, 360, 5)):
                    Rz = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
                    out.append(Rz @ base)
            else:
                out.append(base)
    return out, deg_axis


def match(src, dst, tol):
    """Rigid transform src -> dst within tol (two-way vertex-to-surface), or None. Returns (R, t, err)."""
    if not (abs(src.area - dst.area) <= 0.01 * dst.area + 1e-8):
        return None
    if np.any(np.abs(src.w - dst.w) > 0.02 * np.maximum(np.abs(dst.w), 1e-12) + 1e-10):
        return None
    cands, deg = candidate_rotations(src, dst)
    if deg == "all" or not cands:
        return None
    best = None
    for R in cands:
        t = dst.c - R @ src.c
        e = np.max(dst.surf_dist(src.sub[:60] @ R.T + t))
        if best is None or e < best[0]:
            best = (e, R, t)
    e, R, t = best
    for _ in range(6):  # ICP refine on nearest vertices
        P = src.sub @ R.T + t
        idx = [dst.kd.find(p)[1] for p in P.tolist()]
        R2, t2 = kabsch(src.sub, dst.co[idx])
        if np.max(dst.surf_dist(src.sub @ R2.T + t2)) <= np.max(dst.surf_dist(P)):
            R, t = R2, t2
        else:
            break
    e1 = np.max(dst.surf_dist(src.co @ R.T + t)) if len(src.co) <= 4000 else np.max(dst.surf_dist(src.sub @ R.T + t))
    if e1 > tol:
        return None
    Ri = R.T
    e2 = np.max(src.surf_dist((dst.co - t) @ Ri.T)) if len(dst.co) <= 4000 else np.max(src.surf_dist((dst.sub - t) @ Ri.T))
    if e2 > tol:
        return None
    return R, t, max(e1, e2)


def island_arrays(A, faces):
    faces = np.sort(faces)
    lt = A.lt[faces]
    loops = np.repeat(A.ls[faces] - np.concatenate(([0], np.cumsum(lt)[:-1])), lt) + np.arange(lt.sum())
    verts, lv_new = np.unique(A.lv[loops], return_inverse=True)
    return faces, lt, loops, verts, lv_new.astype(np.int32)


objs = [o for o in sc.objects if o.type == 'MESH' and len(o.data.polygons)]
if not EXPLODE_ALL:
    objs = [o for o in objs if cfg.part(o.name) in getattr(cfg, "EXPLODE", ())]
canon = collections.defaultdict(list)   # key -> [(mesh, Shape of the canonical piece, its centroid)]
errs = []
info = {"exploded": {}, "instances": 0, "unique": 0, "pieces": 0}
stats = collections.Counter()
done_meshes = {}
for n_o, o in enumerate(sorted(objs, key=lambda o: o.name)):
    me = o.data
    t = time.time()
    if me.name in done_meshes:  # objects sharing a mesh: same pieces, same local transforms
        recipe = done_meshes[me.name]
    else:
        A = X.MeshArrays(me)
        isl = sorted(X.islands(me), key=lambda f: (-len(f), int(np.min(f))))
        recipe = []
        for f in isl:
            faces, lt, loops, verts, lv_new = island_arrays(A, f)
            P = A.co[verts].astype(np.float64)
            mi = A.attrs.get("material_index")
            mloc = mi[2][faces] if mi else np.zeros(len(faces), np.int32)
            mats = tuple((A.me.materials[i].name if i < len(A.me.materials) and A.me.materials[i] else "-") for i in mloc)
            key = tuple(sorted(set(mats)))
            shp = Shape(P, lv_new, lt)
            hit = None
            for (cm, csh, c0) in canon[key]:
                r = match(csh, shp, TOL)
                if r is not None:
                    R, tt = r[0], r[1]
                    # canonical data = csh points - c0; piece = R @ (data + c0) + tt
                    hit = (cm, R, tt + R @ c0)
                    errs.append(r[2])
                    break
            if hit:
                cm, R, tt = hit
                M = Matrix.Identity(4)
                for i in range(3):
                    for j in range(3):
                        M[i][j] = R[i, j]
                    M[i][3] = tt[i]
                recipe.append((cm, M))
                stats["instance"] += 1
            else:
                c0 = shp.c
                pm = X.piece_mesh(A, f, f"{me.name}~{len(recipe):03d}")
                pm.transform(Matrix.Translation((-c0).tolist()))
                canon[key].append((pm, shp, c0))
                recipe.append((pm, Matrix.Translation(c0.tolist())))
                stats["unique"] += 1
        done_meshes[me.name] = recipe
    names = []
    for k, (pm, M) in enumerate(recipe):
        po = bpy.data.objects.new(f"{o.name}~{k:03d}", pm)
        for c in o.users_collection:
            c.objects.link(po)
        po.parent = o
        po.matrix_parent_inverse.identity()
        po.matrix_basis = M
        po["piece_of"] = o.name
        names.append(po.name)
    o["exploded_from"] = me.name
    empty = bpy.data.meshes.new(me.name + "~exploded")
    old = o.data
    o.data = empty
    info["exploded"][o.name] = {"mesh": old.name, "pieces": names}
    stats["pieces"] += len(names)
    if n_o % 50 == 0 or time.time() - t > 5:
        print(f"EXPLODE_INST {n_o}/{len(objs)} {o.name}: {len(names)} pieces ({time.time() - t:.1f}s) total {dict(stats)}", flush=True)
for o in objs:
    m = bpy.data.meshes.get(info["exploded"][o.name]["mesh"])
    if m and m.users == 0:
        bpy.data.meshes.remove(m)
info.update(instances=stats["instance"], unique=stats["unique"], pieces=stats["pieces"],
            match_err_mm=[round(float(np.max(errs)) * 1000, 4) if errs else 0, round(float(np.median(errs)) * 1000, 4) if errs else 0])
json.dump(info, open(INFO, "w"), indent=0)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=False)
nm = len([o for o in sc.objects if o.type == 'MESH' and len(o.data.polygons)])
ut = sum(sum(len(p.vertices) - 2 for p in m.polygons) for m in bpy.data.meshes if m.users)
print(f"EXPLODE_INST done: {nm} piece objects, {len(bpy.data.meshes)} meshes, unique tris {ut}, {dict(stats)}, "
      f"{time.time() - T0:.1f}s", flush=True)
