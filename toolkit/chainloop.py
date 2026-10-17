# Chain as ONE loop (owner rule, 2026-10-14: "chains should be one loop instead of links"): a fused CAD chain mesh
# (every link, plate and pin in one mesh) becomes one closed band along the chain path, like a toothless belt: the
# chain's side-plate height across the run, its full width along the pins, no links, plates, pins or rollers.
#   1. plane of the run by PCA (smallest axis = across the chain), surface samples projected into it and rasterised
#      (0.12 mm), closed by 1 mm (gaps between plates and rollers)
#   2. outside = flood from the border, the inner hole = flood from points >= 3 mm from the chain; both boundaries traced
#   3. centreline = midpoint of every outer-boundary point and its nearest inner-boundary point, averaged over one
#      pitch (the plates' round ends and waists repeat every half pitch), Douglas-Peucker 0.15 mm
#   4. band = centreline offset +- half the plate height (90th percentile of the outer-inner distance = height at the
#      pins), mitred, extruded over the chain's width; walls smooth, cap edges sharp
# Used through cfg.BUILDERS (a robot config's FORCE_METHOD "chain loop"); returns (mesh, info) or (None, {"error"}).
import math
import numpy as np
import bpy
import geo, build
from mathutils import Vector, kdtree

GRID_MM = 0.12
CLOSE_MM = 1.0
SEED_MM = 3.0
DP_MM = 0.15


def _shift_or(m, cross):
    out = m.copy()
    out[1:, :] |= m[:-1, :]; out[:-1, :] |= m[1:, :]
    out[:, 1:] |= m[:, :-1]; out[:, :-1] |= m[:, 1:]
    if not cross:
        out[1:, 1:] |= m[:-1, :-1]; out[:-1, :-1] |= m[1:, 1:]
        out[1:, :-1] |= m[:-1, 1:]; out[:-1, 1:] |= m[1:, :-1]
    return out


def dilate(m, r):
    """Octagonal approximation of a disk dilation by r pixels (alternating 4- and 8-neighbour steps)."""
    for k in range(r):
        m = _shift_or(m, cross=k % 2 == 0)
    return m


def erode(m, r):
    return ~dilate(~m, r)


def flood(seed, mask):
    """Pixels of mask connected to seed (iterative 4-neighbour growth)."""
    cur = seed & mask
    while True:
        nxt = _shift_or(cur, cross=True) & mask
        if (nxt == cur).all():
            return cur
        cur = nxt


def trace(region):
    """Longest closed boundary loop of a boolean region, as (K, 2) pixel-edge midpoints (row, col), region on the left."""
    R = np.pad(region, 1)
    nxt = {}
    # directed unit edges between pixel corners with the region on the left
    ys, xs = np.nonzero(R[1:, :] != R[:-1, :])        # horizontal edges between row y and y+1 (corner row y+1)
    for y, x in zip(ys, xs):
        if R[y, x]:   # region above -> edge runs towards -x along corner row y+1
            a, b = (y + 1, x + 1), (y + 1, x)
        else:
            a, b = (y + 1, x), (y + 1, x + 1)
        nxt.setdefault(a, []).append(b)
    ys, xs = np.nonzero(R[:, 1:] != R[:, :-1])        # vertical edges between col x and x+1 (corner col x+1)
    for y, x in zip(ys, xs):
        if R[y, x]:   # region on the left (-x side) -> edge runs towards +y
            a, b = (y, x + 1), (y + 1, x + 1)
        else:
            a, b = (y + 1, x + 1), (y, x + 1)
        nxt.setdefault(a, []).append(b)
    used = set()
    loops = []
    for start in list(nxt):
        for b0 in nxt[start]:
            if (start, b0) in used:
                continue
            loop = [start]
            a, b = start, b0
            while (a, b) not in used:
                used.add((a, b))
                loop.append(b)
                outs = [c for c in nxt.get(b, []) if (b, c) not in used]
                if not outs:
                    break
                if len(outs) > 1:  # pinch corner: take the sharpest right turn so touching regions stay apart
                    d0 = (b[0] - a[0], b[1] - a[1])
                    outs.sort(key=lambda c: -((d0[0] * (c[1] - b[1]) - d0[1] * (c[0] - b[0]))))
                a, b = b, outs[0]
            loops.append(loop)
    best = max(loops, key=len)
    p = np.array(best[:-1], float)
    return (p + np.roll(p, -1, axis=0)) / 2 - 1.0  # edge midpoints, undo the padding


def smooth_closed(p, k):
    if k <= 1:
        return p
    pad = np.concatenate([p[-k:], p, p[:k]])
    c = np.cumsum(np.vstack([np.zeros((1, 2)), pad]), axis=0)
    return (c[2 * k + 1:] - c[:-2 * k - 1]) / (2 * k + 1)


def resample_closed(p, step):
    q = np.vstack([p, p[:1]])
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))])
    t = np.arange(0, s[-1], step)
    return np.column_stack([np.interp(t, s, q[:, 0]), np.interp(t, s, q[:, 1])])


def dp_closed(p, tol):
    """Douglas-Peucker on a closed polyline (split at the two farthest-apart points)."""
    i0 = 0
    i1 = int(np.argmax(np.linalg.norm(p - p[0], axis=1)))

    def dp(seg):
        if len(seg) < 3:
            return [0, len(seg) - 1]
        a, b = seg[0], seg[-1]
        ab = b - a
        L = np.linalg.norm(ab)
        if L < 1e-12:
            d = np.linalg.norm(seg - a, axis=1)
        else:
            d = np.abs(ab[0] * (seg[:, 1] - a[1]) - ab[1] * (seg[:, 0] - a[0])) / L
        k = int(np.argmax(d))
        if d[k] <= tol:
            return [0, len(seg) - 1]
        l = dp(seg[:k + 1])
        r = dp(seg[k:])
        return l[:-1] + [k + j for j in r]
    A = dp(p[i0:i1 + 1])
    B = dp(np.vstack([p[i1:], p[:1]]))
    idx = A[:-1] + [i1 + j for j in B[:-1]]
    return p[idx]


def build_chain_loop(name, me):
    co, tri, _ = geo.mesh_arrays(me)
    if len(tri) < 50:
        return None, {"error": "too small"}
    c = co.mean(0)
    w, V = np.linalg.eigh(np.cov((co - c).T))
    n = V[:, 0]; u = V[:, 2]; v = np.cross(n, u)
    t = (co - c) @ n
    t0, t1 = float(t.min()), float(t.max())
    pts, area = geo.sample_surface(co, tri, 1)
    g = GRID_MM * 1e-3
    ns = int(min(4e6, max(3e5, 2.0 * area.sum() / g ** 2)))
    pts, _ = geo.sample_surface(co, tri, ns)
    P = np.column_stack([(pts - c) @ u, (pts - c) @ v])
    lo = P.min(0) - 5e-3
    ij = np.floor((P - lo) / g).astype(np.int64)
    H, W = ij.max(0) + 6
    B = np.zeros((H, W), bool)
    B[ij[:, 0], ij[:, 1]] = True
    rc = max(1, int(round(CLOSE_MM / GRID_MM)))
    B = erode(dilate(B, rc), rc)
    border = np.zeros_like(B)
    border[0, :] = border[-1, :] = border[:, 0] = border[:, -1] = True
    outside = flood(border & ~B, ~B)
    holes = ~B & ~outside
    seeds = erode(holes, int(round(SEED_MM / GRID_MM)))
    if not seeds.any():
        return None, {"error": "no inner opening (open chain run?)"}
    inner = flood(seeds, holes)
    band = ~outside & ~inner
    O = (trace(~outside) + 0.5) * g + lo   # pixel-edge midpoints -> plane coordinates (m)
    I = (trace(inner) + 0.5) * g + lo
    O = smooth_closed(resample_closed(O, 0.1e-3), 3)
    I = smooth_closed(resample_closed(I, 0.1e-3), 3)
    kd = kdtree.KDTree(len(I))
    for i, q in enumerate(I):
        kd.insert((q[0], q[1], 0.0), i)
    kd.balance()
    mid, dist = [], []
    for q in O:
        (x, y, _), _, d = kd.find((q[0], q[1], 0.0))
        mid.append(((q[0] + x) / 2, (q[1] + y) / 2)); dist.append(d)
    mid = np.array(mid); dist = np.array(dist)
    h = float(np.percentile(dist, 90))
    C = resample_closed(mid, 0.25e-3)
    pitch_pts = max(1, int(round(6.35e-3 / 0.25e-3 / 2)))  # one-pitch window (half on each side)
    C = smooth_closed(C, pitch_pts)
    C = dp_closed(C, DP_MM * 1e-3)
    m = len(C)
    if m < 3:
        return None, {"error": "degenerate centreline"}
    # mitred offsets
    seg = np.roll(C, -1, axis=0) - C
    seg /= np.linalg.norm(seg, axis=1)[:, None]
    nrm = np.column_stack([-seg[:, 1], seg[:, 0]])
    nv = nrm + np.roll(nrm, 1, axis=0)
    nv /= np.linalg.norm(nv, axis=1)[:, None]
    scale = 1.0 / np.clip((nv * nrm).sum(1), 0.3, 1.0)
    off = nv * (h / 2 * scale)[:, None]
    A2, B2 = C + off, C - off
    to3 = lambda q2, tt: c + u * q2[0] + v * q2[1] + n * tt
    verts = []
    for i in range(m):
        verts += [to3(A2[i], t0), to3(A2[i], t1), to3(B2[i], t0), to3(B2[i], t1)]
    faces, smooth, sharp = [], set(), set()
    for i in range(m):
        j = (i + 1) % m
        a0, a1, b0, b1 = 4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3
        c0, c1, d0, d1 = 4 * j, 4 * j + 1, 4 * j + 2, 4 * j + 3
        smooth.add(len(faces)); faces.append((a0, c0, c1, a1))     # wall on the +offset side
        smooth.add(len(faces)); faces.append((b1, d1, d0, b0))     # wall on the -offset side
        faces.append((a0, b0, d0, c0))                            # cap at t0
        faces.append((a1, c1, d1, b1))                            # cap at t1
        sharp |= {frozenset((a0, c0)), frozenset((a1, c1)), frozenset((b0, d0)), frozenset((b1, d1))}
    new = build.new_mesh_from(name, verts, faces, smooth, sharp)
    import bmesh
    bm = bmesh.new(); bm.from_mesh(new)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(new); bm.free(); new.update()
    build.fix_inverted_shells(new)
    e1, e2 = build.fit_error(me, new, 4000)
    return new, {"method": "chain loop", "height_mm": round(h * 1000, 2), "width_mm": round((t1 - t0) * 1000, 2),
                 "segments": m, "err_mm": [round(e1, 2), round(e2, 2)],
                 "band_area_mm2": round(float(band.sum()) * GRID_MM ** 2, 1)}
