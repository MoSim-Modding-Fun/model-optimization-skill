# Rigid alignment of a library asset onto a CAD part (PCA starts + ICP), numpy only.
import bpy, math, itertools
import numpy as np
from mathutils import Matrix
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geo  # noqa: E402
import paths  # noqa: E402

LIB = paths.components(required=False)  # None until the asset library is configured (install.sh)


def append_object(blend, name):
    with bpy.data.libraries.load(blend, link=False) as (src, dst):
        if name not in src.objects:
            raise KeyError(f"{name} not in {blend}")
        dst.objects = [name]
    return dst.objects[0]


def world_mesh_arrays(obj, me=None):
    me = me or obj.data
    co, tri, mat = geo.mesh_arrays(me)
    M = np.array(obj.matrix_world)
    co = co @ M[:3, :3].T + M[:3, 3]
    return co, tri


def _nn(src, dst, chunk=2000):
    out_d = np.empty(len(src))
    out_i = np.empty(len(src), np.int64)
    dn = (dst ** 2).sum(1)
    for s in range(0, len(src), chunk):
        a = src[s:s + chunk]
        d2 = (a ** 2).sum(1)[:, None] + dn[None, :] - 2 * a @ dst.T
        i = np.argmin(d2, axis=1)
        out_i[s:s + chunk] = i
        out_d[s:s + chunk] = np.sqrt(np.maximum(d2[np.arange(len(a)), i], 0))
    return out_d, out_i


def _kabsch(P, Q):
    cp, cq = P.mean(0), Q.mean(0)
    H = (P - cp).T @ (Q - cq)
    U, S, Vt = np.linalg.svd(H)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T
    return R, cq - R @ cp


def _rotations24():
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            P = np.zeros((3, 3))
            for i in range(3):
                P[i, perm[i]] = signs[i]
            if np.linalg.det(P) > 0:
                out.append(P)
    return out


def icp(S, T, R, t, iters):
    for _ in range(iters):
        X = S @ R.T + t
        d, i = _nn(X, T)
        keep = d <= np.percentile(d, 90)
        dR, dt = _kabsch(X[keep], T[i[keep]])
        R, t = dR @ R, dR @ t + dt
    return R, t


def chamfer(S, T, R, t):
    X = S @ R.T + t
    a, _ = _nn(X, T)
    b, _ = _nn(T, X)
    return float(a.mean()), float(b.mean())


def align(src_co, src_tri, dst_co, dst_tri, rng=None, tiebreak=None):
    """Returns (R, t, (mean src->dst, mean dst->src)) mapping src points onto dst.
    tiebreak="bbox" (opt-in): among near-ties first keep the fits whose world bbox extents match the target's (a
    1-stage MAXPlanetary scored 1.70 mm turned 90 deg about its output axis vs 1.79 mm in the CAD's pose)."""
    rng = rng or np.random.default_rng(1)
    S, _ = geo.sample_surface(src_co, src_tri, 1500, rng)
    T, _ = geo.sample_surface(dst_co, dst_tri, 6000, rng)
    cS, aS = geo.pca_axes(S)
    cT, aT = geo.pca_axes(T)
    if np.linalg.det(aS) < 0:
        aS[2] *= -1
    if np.linalg.det(aT) < 0:
        aT[2] *= -1
    starts = []
    inits = [aT.T @ P @ aS for P in _rotations24()] + list(_rotations24())
    for R0 in inits:
        if np.linalg.det(R0) < 0:
            continue
        t0 = cT - R0 @ cS
        R1, t1 = icp(S, T, R0, t0, 8)
        starts.append((sum(chamfer(S, T, R1, t1)), R1, t1))
    starts.sort(key=lambda x: x[0])
    S2, _ = geo.sample_surface(src_co, src_tri, 4000, rng)
    T2, _ = geo.sample_surface(dst_co, dst_tri, 15000, rng)
    refined = []
    for _, R, t in starts[:8 if tiebreak == "bbox" else 4]:  # bbox tie-break: the right pose can rank 4th-8th
        R, t = icp(S2, T2, R, t, 30)
        refined.append((R, t, chamfer(S2, T2, R, t)))
    # flat boxes (Pigeon, roboRIO...) fit almost equally well flipped; CAD and library usually share
    # the same local axes, so among near-ties prefer the rotation closest to identity
    lo = min(sum(c) for _, _, c in refined)
    near = [r for r in refined if sum(r[2]) <= lo * 1.10]
    if tiebreak == "bbox":
        ext_t = np.ptp(dst_co, axis=0)
        diff = [float(np.abs(np.ptp(src_co @ R.T + t, axis=0) - ext_t).sum()) for R, t, _ in near]
        near = [r for r, d_ in zip(near, diff) if d_ <= min(diff) + 1e-3]
    return max(near, key=lambda r: np.trace(r[0]))


def to_matrix(R, t):
    M = Matrix.Identity(4)
    for i in range(3):
        for j in range(3):
            M[i][j] = R[i, j]
        M[i][3] = t[i]
    return M
