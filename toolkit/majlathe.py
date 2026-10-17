# "Majority" lathe: a solid of revolution whose (radius, height) cells are solid where MOST of the circumference
# is material. Rays along the axis at every radius (0.25 mm) and 36 angles give the occupied height intervals; a cell is
# solid when > 50 % of the angles are; the boundary of that region is traced, Douglas-Peucker simplified and revolved.
# Thin ribs, spokes and bolt holes (a small share of the circumference) drop out; a cup, its central boss, the web and
# the rim stay, which per-height radius profiles cannot do: they close a cup with a boss in it (a 4" Stealth wheel
# hub, flagged open-vs-closed by the review). Used through cfg.BUILDERS; returns (mesh, info) or (None, {"error"}).
import math
import numpy as np
import bpy, bmesh
import geo, build, chainloop
from mathutils import Vector
from mathutils.bvhtree import BVHTree

CELL_MM = 0.25
ANGLES = 36


def _axis(me):
    co, tri, _ = geo.mesh_arrays(me)
    pts, _ = geo.sample_surface(co, tri, 20000)
    best = (9, None, None)
    for d in geo.candidate_axes(pts):
        s, c = geo.revolve_score(pts, d)
        if s < best[0]:
            best = (s, d, c)
    return co, tri, best


def build_majority_lathe(name, me, frac=0.5, tol_mm=0.3, nseg=None):
    co, tri, (score, d, c) = _axis(me)
    d = np.asarray(d, float); d /= np.linalg.norm(d); c = np.asarray(c, float)
    u = np.cross(d, [1.0, 0, 0] if abs(d[0]) < 0.9 else [0, 1.0, 0]); u /= np.linalg.norm(u); w = np.cross(d, u)
    rel = co - c
    h = rel @ d
    r = np.linalg.norm(rel - np.outer(h, d), axis=1)
    h0, h1, R = float(h.min()), float(h.max()), float(r.max())
    g = CELL_MM * 1e-3
    nr, nh = int(math.ceil(R / g)) + 2, int(math.ceil((h1 - h0) / g)) + 3
    bvh = BVHTree.FromPolygons([Vector(x) for x in co], [tuple(t) for t in tri])
    occ = np.zeros((nh, nr), np.float32)
    dv = Vector(d)
    for j in range(nr):
        rj = (j + 0.5) * g
        if rj > R:
            break
        for k in range(ANGLES):
            t = 2 * math.pi * (k + 0.5) / ANGLES
            o = c + d * (h0 - 1e-3) + rj * (math.cos(t) * u + math.sin(t) * w)
            hits, start, guard = [], Vector(o), 0
            while guard < 64:
                loc, nrm, idx, dist = bvh.ray_cast(start, dv, (h1 - h0) + 3e-3)
                if loc is None:
                    break
                hits.append(float((np.array(loc) - c) @ d))
                start = loc + dv * 1e-6; guard += 1
            # parity: entering / leaving pairs (closed CAD solids; coincident hits from welded seams are merged)
            hs = []
            for x in sorted(hits):
                if not hs or x - hs[-1] > 2e-6:
                    hs.append(x)
            for a, b in zip(hs[0::2], hs[1::2]):
                i0 = int(max(0, math.floor((a - h0) / g) + 1)); i1 = int(min(nh, math.ceil((b - h0) / g) + 1))
                occ[i0:i1, j] += 1.0 / ANGLES
    solid = occ > frac
    if not solid.any():
        return None, {"error": "no majority-solid cells"}
    # region in (h, r) image space -> boundary loop (chainloop.trace: pixel-edge midpoints), back to metres
    loop = chainloop.trace(solid)  # (row = h index, col = r index)
    # traced points are cell corners: column x = radius x * g; row y = height h0 + (y - 1) * g (row 0 is the margin)
    P = np.column_stack([loop[:, 1] * g, (loop[:, 0] - 1) * g + h0])  # (r, h)
    P[:, 0] = np.where(P[:, 0] < 0.5 * g, 0.0, P[:, 0])  # cells touching the axis -> on the axis
    P = chainloop.dp_closed(P, tol_mm * 1e-3)
    m = len(P)
    if m < 3:
        return None, {"error": "degenerate profile"}
    n = nseg or geo.ngon_segments(float(P[:, 0].max()) * 1000)
    verts, ring = [], []
    for (rr, hh) in P:
        base = c + d * hh
        if rr <= 0:
            ring.append([len(verts)]); verts.append(tuple(base))
        else:
            ids = []
            for k in range(n):
                t = 2 * math.pi * k / n
                ids.append(len(verts)); verts.append(tuple(base + rr * (math.cos(t) * u + math.sin(t) * w)))
            ring.append(ids)
    faces = []
    for i in range(m):
        A, B = ring[i], ring[(i + 1) % m]
        if len(A) == 1 and len(B) == 1:
            continue
        for k in range(n):
            k2 = (k + 1) % n
            if len(A) == 1:
                faces.append((A[0], B[k], B[k2]))
            elif len(B) == 1:
                faces.append((A[k], B[0], A[k2]))
            else:
                faces.append((A[k], B[k], B[k2], A[k2]))
    new = build.new_mesh_from(name, verts, faces, set(range(len(faces))), None)
    bm = bmesh.new(); bm.from_mesh(new)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(new); bm.free(); new.update()
    build.fix_inverted_shells(new)
    e1, e2 = build.fit_error(me, new, 4000)
    return new, {"method": "cylinder rebuild", "note": "majority lathe (majlathe.py)", "profile_pts": m, "sides": n,
                 "round_score": round(float(score), 4), "err_mm": [round(e1, 2), round(e2, 2)],
                 "open_edges": build.open_edges(new)}
