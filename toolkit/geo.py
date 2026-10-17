# Geometry helpers for the simplification pipeline (run inside Blender's Python).
import numpy as np
import math


def mesh_arrays(me):
    """Vertices (N,3) and triangle vertex indices (T,3) plus material index per tri, in mesh-local space."""
    me.calc_loop_triangles()
    co = np.empty(len(me.vertices) * 3, np.float64)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    tri = np.empty(len(me.loop_triangles) * 3, np.int64)
    me.loop_triangles.foreach_get("vertices", tri)
    tri = tri.reshape(-1, 3)
    mat = np.empty(len(me.loop_triangles), np.int64)
    me.loop_triangles.foreach_get("material_index", mat)
    return co, tri, mat


def sample_surface(co, tri, n, rng=None):
    rng = rng or np.random.default_rng(0)
    a, b, c = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    tot = area.sum()
    if tot <= 0:
        return co.copy(), area
    idx = rng.choice(len(tri), size=n, p=area / tot)
    u = rng.random(n)
    v = rng.random(n)
    flip = u + v > 1
    u[flip] = 1 - u[flip]
    v[flip] = 1 - v[flip]
    pts = a[idx] + (b[idx] - a[idx]) * u[:, None] + (c[idx] - a[idx]) * v[:, None]
    return pts, area


def pca_axes(pts):
    ctr = pts.mean(0)
    w, v = np.linalg.eigh(np.cov((pts - ctr).T))
    order = np.argsort(w)[::-1]
    return ctr, v[:, order].T  # rows = axes, largest variance first


def perp_basis(d):
    d = d / np.linalg.norm(d)
    t = np.array([1.0, 0, 0]) if abs(d[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(d, t)
    u /= np.linalg.norm(u)
    w = np.cross(d, u)
    return u, w


def snap_axis(d):
    """Snap a direction to the nearest local axis when within ~3 degrees (CAD parts are usually axis aligned)."""
    d = d / np.linalg.norm(d)
    k = int(np.argmax(np.abs(d)))
    if abs(d[k]) > 0.9986:
        e = np.zeros(3)
        e[k] = np.sign(d[k])
        return e
    return d


def candidate_axes(pts):
    _, ax = pca_axes(pts)
    cands = [np.eye(3)[i] for i in range(3)]
    for a in ax:
        a = snap_axis(a)
        if all(abs(np.dot(a, c)) < 0.9986 for c in cands):
            cands.append(a)
    return cands


def revolve_score(pts, d, nsec=24, nbins=24):
    """How rotationally symmetric the outer envelope is about axis d (through the bbox centre in the
    perpendicular plane). Returns (score, centre_point). 0 = perfect cylinder-like."""
    u, w = perp_basis(d)
    pu, pw, ph = pts @ u, pts @ w, pts @ d
    cu = 0.5 * (pu.min() + pu.max())
    cw = 0.5 * (pw.min() + pw.max())
    ru, rw = pu - cu, pw - cw
    r = np.hypot(ru, rw)
    th = np.arctan2(rw, ru)
    sec = ((th + math.pi) / (2 * math.pi) * nsec).astype(int) % nsec
    hb = np.clip(((ph - ph.min()) / max(np.ptp(ph), 1e-9) * nbins).astype(int), 0, nbins - 1)
    key = hb * nsec + sec
    smax = np.full(nbins * nsec, -1.0)
    np.maximum.at(smax, key, r)
    smax = smax.reshape(nbins, nsec)
    scores, weights = [], []
    for b in range(nbins):
        row = smax[b]
        if (row >= 0).sum() < nsec * 0.9:
            continue
        row = row[row >= 0]
        m = row.mean()
        if m <= 0:
            continue
        scores.append(row.std() / m)
        weights.append(m)
    if not scores:
        return 9.0, None
    centre = d * 0 + u * cu + w * cw
    return float(np.average(scores, weights=weights)), centre


def axis_levels(co, d, tol=2e-5):
    """Distinct vertex levels along d (clustered)."""
    h = np.sort(co @ d)
    lv = [h[0]]
    for x in h[1:]:
        if x - lv[-1] > tol:
            lv.append(x)
    return lv


def douglas_peucker(pts, tol, closed=False):
    pts = np.asarray(pts, float)
    if len(pts) < 3:
        return pts
    if closed:
        # split at the two farthest-apart points
        i0 = 0
        dists = np.linalg.norm(pts - pts[i0], axis=1)
        i1 = int(np.argmax(dists))
        a = douglas_peucker(np.vstack([pts[i0:i1 + 1]]), tol)
        b = douglas_peucker(np.vstack([pts[i1:], pts[:1]]), tol)
        return np.vstack([a[:-1], b[:-1]])
    keep = np.zeros(len(pts), bool)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        s, e = stack.pop()
        if e <= s + 1:
            continue
        a, b = pts[s], pts[e]
        ab = b - a
        L = np.linalg.norm(ab)
        seg = pts[s + 1:e]
        if L < 1e-12:
            dd = np.linalg.norm(seg - a, axis=1)
        else:
            dd = np.abs(ab[0] * (seg[:, 1] - a[1]) - ab[1] * (seg[:, 0] - a[0])) / L
        k = int(np.argmax(dd))
        if dd[k] > tol:
            m = s + 1 + k
            keep[m] = True
            stack.append((s, m))
            stack.append((m, e))
    return pts[keep]


def poly_area(p):
    p = np.asarray(p)
    x, y = p[:, 0], p[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def fit_circle(p):
    p = np.asarray(p, float)
    A = np.c_[2 * p, np.ones(len(p))]
    b = (p ** 2).sum(1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = sol[:2]
    r = math.sqrt(max(sol[2] + c @ c, 0))
    dev = np.abs(np.linalg.norm(p - c, axis=1) - r).max()
    return c, r, dev


def ngon_segments(r_mm):
    r_mm += 0.1  # measured radii come out a hair small (6.00 mm spacer measured 5.95)
    if r_mm < 6: return 8
    if r_mm < 12: return 16
    if r_mm < 20: return 24
    if r_mm < 35: return 32
    if r_mm < 45: return 48
    return 64
