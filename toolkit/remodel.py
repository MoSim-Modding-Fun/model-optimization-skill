# Remodel of machined and sheet-metal parts: the part's cross-sections stacked along one axis (holes
# along that axis come with the sections), then every hole across that axis re-cut by n-gon / polygon cutters in ONE
# exact boolean. For gearbox faces, block mounts, servo blocks, U-channels (axis = length: the C section extruded, the
# wall hole patterns cut) and grid sheets (flat: axis = thickness, every hole in the section; bent: axis = bend axis,
# holes cut along the local surface normal). Collapse turned these into lumps with triangular holes.
#   module: new_me, info = remodel.remodel(name, cad_mesh, axis="auto", ...)
#   test:   blender -b file.blend --python remodel.py -- "<object>" <axis x|y|z|auto|slabs> out.blend
import bpy, bmesh, math, collections, os, sys
import numpy as np
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geo, build

MM = 1e-3
AXES = {"x": np.array([1.0, 0, 0]), "y": np.array([0, 1.0, 0]), "z": np.array([0, 0, 1.0])}


def default_sides(r_mm):
    """Hole / section circle sides: hexagons for bolt and pattern holes (r < 3 mm), then the owner's radius tiers."""
    return 6 if r_mm < 3 else geo.ngon_segments(r_mm)


def _tris(me):
    co, tri, _ = geo.mesh_arrays(me)
    a, b, c = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
    cr = np.cross(b - a, c - a)
    ar = np.linalg.norm(cr, axis=1) / 2
    n = cr / np.maximum(2 * ar, 1e-30)[:, None]
    return co, tri, n, ar, (a + b + c) / 3


def auto_axis(me):
    """The axis with the fewest section levels between the ends (U-channel: its length; bent sheet: the bend axis;
    flat sheet: any in-plane axis or its thickness), ties -> the larger share of area facing the axis (flat sheet:
    the thickness, so every hole comes with the sections instead of being cut)."""
    co, tri, n, ar, cen = _tris(me)
    A = ar.sum()
    score = {}
    for k, d in AXES.items():
        lv = _levels(co, tri, n, ar, cen, d)
        cap = ar[np.abs(n @ d) > 0.99].sum() / A
        score[k] = (len(lv), -round(cap, 2))
    return min(score, key=score.get)


def _cap_regions(co, tri, n, ar, cen, d, min_area=1.5 * MM * MM, min_width=1.0 * MM):
    """Triangles facing along d that belong to a real flat region (connected, area and in-plane width big enough):
    the facets of a hole wall that happen to face d are thin strips and must not start a new section level."""
    m = np.nonzero(np.abs(n @ d) > 0.995)[0]
    if len(m) == 0:
        return m
    h = cen[m] @ d
    edge_owner = collections.defaultdict(list)
    for k, t in enumerate(m):
        a, b, c = tri[t]
        for e in ((a, b), (b, c), (c, a)):
            edge_owner[(min(e), max(e))].append(k)
    parent = list(range(len(m)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]; x = parent[x]
        return x
    for ks in edge_owner.values():
        for k in ks[1:]:
            if abs(h[k] - h[ks[0]]) < 1e-5:
                parent[find(k)] = find(ks[0])
    groups = collections.defaultdict(list)
    for k in range(len(m)):
        groups[find(k)].append(k)
    u, w = geo.perp_basis(d)
    keep = []
    for ks in groups.values():
        ts = m[ks]
        if ar[ts].sum() < min_area:
            continue
        P = co[tri[ts].ravel()]
        span = np.ptp(np.c_[P @ u, P @ w], axis=0)
        if span.min() < min_width:
            continue
        keep.extend(ts.tolist())
    return np.array(keep, int)


def bend_axis(me):
    """Bent sheet (single curvature): the direction every sheet-face normal is perpendicular to (smallest eigenvector of
    the area-weighted normal tensor over the faces within 35 deg of the main facing; hole walls and edges left out)."""
    co, tri, n, ar, cen = _tris(me)
    M = (n * ar[:, None]).T @ n
    main = np.linalg.eigh(M)[1][:, -1]
    big = np.abs(n @ main) > math.cos(math.radians(35))
    M2 = (n[big] * ar[big, None]).T @ n[big]
    return np.linalg.eigh(M2)[1][:, 0]


def _levels(co, tri, n, ar, cen, d, merge=0.25 * MM, min_area=0.3 * MM * MM):
    h = co @ d
    lo, hi = float(h.min()), float(h.max())
    m = _cap_regions(co, tri, n, ar, cen, d)
    if len(m) == 0:
        return [lo, hi]
    hs, ws = (cen[m] @ d), ar[m]
    order = np.argsort(hs)
    hs, ws = hs[order], ws[order]
    lv, cur, cw = [], [], []
    for x, w_ in zip(hs, ws):
        if cur and x - cur[-1] > merge:
            if sum(cw) > min_area:
                lv.append(float(np.average(cur, weights=cw)))
            cur, cw = [], []
        cur.append(x); cw.append(w_)
    if cur and sum(cw) > min_area:
        lv.append(float(np.average(cur, weights=cw)))
    lv = [x for x in lv if lo + merge < x < hi - merge]
    return [lo] + lv + [hi]


def _depths(loops):
    dep = []
    for i, L in enumerate(loops):
        p = L[0]
        dep.append(sum(1 for j, M in enumerate(loops) if j != i and build.point_in_poly(p, M)))
    return dep


def region_area(loops):
    dep = _depths(loops)
    return sum((1 if k % 2 == 0 else -1) * abs(geo.poly_area(L)) for L, k in zip(loops, dep))


def _simplify(loops, tol, sides, seen, circ_cache):
    out = []
    for L in loops:
        L = np.asarray(L, float)
        A = abs(geo.poly_area(L))
        if A < (0.5 * MM) ** 2 or len(L) < 3:
            continue
        c, r, dev = geo.fit_circle(L)
        # 5% of r: a nubbed tread (bumps ~2%) is still a circle, an 8 mm REX (rounded hex) bore (~7%) is not
        if r < 40 * MM and dev < 0.05 * r and abs(A - math.pi * r * r) < 0.2 * math.pi * r * r:
            key = (round(c[0] / (0.05 * MM)), round(c[1] / (0.05 * MM)), round(r / (0.05 * MM)))
            if key not in circ_cache:  # the same hole on every section gets the same n-gon
                n = sides(r / MM)
                ang = np.arange(n) * 2 * math.pi / n + math.pi / n
                circ_cache[key] = np.c_[c[0] + r * np.cos(ang), c[1] + r * np.sin(ang)]
            out.append(circ_cache[key])
            continue
        L = build._same_outline(build._canonical_loop(L), seen, tol=0.05 * MM)
        s = build._simplify_loop(L, tol, circle_sides=sides)
        if s is not None and abs(geo.poly_area(s)) > (0.5 * MM) ** 2:
            out.append(s)
    return out


def _sig(loops):
    return tuple(sorted((round(abs(geo.poly_area(L)) * 1e8), round(float(L[:, 0].mean()) * 1e5),
                         round(float(L[:, 1].mean()) * 1e5), len(L)) for L in loops))


def free_heights(holes, d, lo, hi, margin=0.3 * MM):
    """Heights along d that no cross hole reaches (middles of the gaps between hole extents, biggest gap first):
    slices there show the full section (a slice through a grid hole row splits a sheet's strip)."""
    iv = []
    for h in holes:
        c = float(h["c"] @ d)
        if "r" in h:
            e = h["r"] * math.sqrt(max(0.0, 1 - float(h["n"] @ d) ** 2)) + margin
        else:
            P = h["poly2d"]; e = float(np.abs(P).max()) + margin
        iv.append((c - e, c + e))
    iv.sort()
    gaps, cur = [], lo
    for a, b in iv:
        if a > cur:
            gaps.append((cur, a))
        cur = max(cur, b)
    if hi > cur:
        gaps.append((cur, hi))
    return [0.5 * (a + b) for a, b in sorted(gaps, key=lambda g: g[0] - g[1])]


def _snap_to(loops, ref, tol):
    """Adjacent stacked runs share their common outline stretches exactly: each vertex within tol of the previous run's
    loops moves onto them (nearest vertex first, else nearest edge point), and the previous run's vertices between two
    snapped neighbours are inserted. The simplified outlines of near-identical sections differ by up to tol, and the union
    showed that as a thin groove / step around the part (the lobes of a kit hub)."""
    R_ = [np.asarray(L, float) for L in ref if len(L) >= 3]
    if not R_:
        return loops
    cum = []
    for A in R_:
        seg = np.linalg.norm(np.roll(A, -1, axis=0) - A, axis=1)
        cum.append((np.r_[0, np.cumsum(seg)[:-1]], seg.sum()))

    def snap(p):
        best = (tol, None, None, None)
        for k, A in enumerate(R_):
            B = np.roll(A, -1, axis=0); AB = B - A
            t = np.clip(((p - A) * AB).sum(1) / np.maximum((AB * AB).sum(1), 1e-18), 0, 1)
            Q = A + AB * t[:, None]; dq = np.linalg.norm(Q - p, axis=1)
            dv = np.linalg.norm(A - p, axis=1); j = int(np.argmin(dv))
            if dv[j] < best[0]:  # a vertex beats an edge point within tol
                best = (dv[j] * 0.5, A[j].copy(), k, cum[k][0][j])
            i = int(np.argmin(dq))
            if dq[i] < best[0]:
                best = (dq[i], Q[i].copy(), k, cum[k][0][i] + t[i] * np.linalg.norm(AB[i]))
        return best[1:]
    out = []
    for L in loops:
        P = np.asarray(L, float)
        sn = [snap(p) for p in P]
        new = []
        for i, p in enumerate(P):
            q, k, sp = sn[i]
            new.append(q if q is not None else p)
            q2, k2, sp2 = sn[(i + 1) % len(P)]
            if q is None or q2 is None or k != k2:
                continue
            Lt = cum[k][1]; fwd = (sp2 - sp) % Lt; bwd = (sp - sp2) % Lt
            chord = np.linalg.norm(q2 - q)
            A, cs = R_[k], cum[k][0]
            if fwd <= bwd and fwd <= 1.5 * chord + 2 * tol:
                mids = sorted((((c - sp) % Lt), j) for j, c in enumerate(cs) if 1e-9 < (c - sp) % Lt < fwd - 1e-9)
            elif bwd < fwd and bwd <= 1.5 * chord + 2 * tol:
                mids = sorted((((sp - c) % Lt), j) for j, c in enumerate(cs) if 1e-9 < (sp - c) % Lt < bwd - 1e-9)
            else:
                mids = []
            new += [A[j].copy() for _, j in mids]
        keep = [new[0]]
        for q in new[1:]:
            if np.linalg.norm(q - keep[-1]) > 1e-7:
                keep.append(q)
        if len(keep) > 2 and np.linalg.norm(keep[0] - keep[-1]) < 1e-7:
            keep.pop()
        if len(keep) >= 3 and abs(geo.poly_area(np.array(keep))) > 1e-10:
            out.append(np.array(keep))
    return out


def stack(name, me, d, tol_mm=0.3, sides=default_sides, max_samples=41, prefer=(), merge_mm=0.25, drop_holes_mm2=0.0,
          snap=False):
    """Runs of identical simplified sections between the cap levels, each extruded; [(mesh, h0, h1, nloops)].
    prefer: heights to try first (free_heights)."""
    co, tri, n, ar, cen = _tris(me)
    lv = _levels(co, tri, n, ar, cen, d, merge=merge_mm * MM)
    seen, circ, runs = [], {}, []
    for h0, h1 in zip(lv[:-1], lv[1:]):
        if h1 - h0 < 0.05 * MM:
            continue
        ns = int(np.clip((h1 - h0) / (1.0 * MM), 3, max_samples))
        fr = np.linspace(0.08, 0.92, ns)
        fr = fr[np.argsort(np.abs(fr - 0.5))]  # middle first: ties go to the middle
        pref = [(x - h0) / (h1 - h0) for x in prefer if h0 + 0.05 * MM < x < h1 - 0.05 * MM][:12]
        if pref:
            fr = np.array(pref + list(fr[:5]))
        best = None
        for f in fr:
            loops, u, w = build.section_loops(me, d, h0 + f * (h1 - h0))
            if not loops:
                continue
            a = region_area(loops)
            if best is None or a > best[0] * (1 + 1e-4):
                best = (a, loops, u, w)
        if best is None:
            continue
        loops = _simplify(best[1], tol_mm * MM, sides, seen, circ)
        if snap and runs and loops:  # opt-in: share the previous run's outline where it is within reach (_snap_to); two
            # outlines simplified at tol each can differ by 2 tol (1 tol left a tapered sliver beside the Hyper Hub's slit)
            loops = _snap_to(loops, runs[-1][0], (snap if isinstance(snap, (int, float)) and snap is not True else 2.0) * tol_mm * MM)
        if drop_holes_mm2 and loops:  # optional: holes (odd nesting depth) under this area are filled
            dep = _depths(loops)
            loops = [L for L, k in zip(loops, dep) if k % 2 == 0 or abs(geo.poly_area(L)) >= drop_holes_mm2 * MM * MM]
        if not loops:
            continue
        sg = _sig(loops)
        if runs and runs[-1][3] == sg and abs(runs[-1][2] - h0) < 1e-7:
            runs[-1][2] = h1
        else:
            runs.append([loops, h0, h1, sg, best[2], best[3]])
    meshes = []
    for i, (loops, h0, h1, sg, u, w) in enumerate(runs):
        meshes.append(build.extrude_loops(f"{name} run{i}", loops, u, w, d, h0, h1))
    return meshes, lv


# ------------------------------------------------------------------ holes across the axis
def _boundary_loops(edges):
    """Closed loops from boundary edges [(a, b)] by vertex walking (pinch vertices: take any unused edge)."""
    adj = collections.defaultdict(list)
    for a, b in edges:
        adj[a].append(b); adj[b].append(a)
    used, loops = set(), []
    for a0, b0 in edges:
        if (a0, b0) in used or (b0, a0) in used:
            continue
        loop, prev, cur = [a0], a0, b0
        used.add((a0, b0))
        ok = False
        while True:
            if cur == a0:
                ok = True
                break
            loop.append(cur)
            nx = [x for x in adj[cur] if (cur, x) not in used and (x, cur) not in used]
            if not nx:
                break
            nxt = nx[0]
            used.add((cur, nxt))
            prev, cur = cur, nxt
        if ok and len(loop) >= 3:
            loops.append(loop)
    return loops


def side_holes(me, d, sides=default_sides, max_hole_mm=30.0, min_poly_area_mm2=2.0, min_depth_mm=0.4, axis_dot=0.25):
    """Holes and pockets that open on faces NOT facing along d: inner boundary loops of the smooth side patches.
    Returns [{"c": centre, "n": outward normal, "r" or "poly2d"+"u"+"w", "depth", "through"}] in mesh coordinates."""
    bm = bmesh.new(); bm.from_mesh(me)
    bm.faces.ensure_lookup_table(); bm.verts.ensure_lookup_table()
    dv = Vector(d)
    side = [f for f in bm.faces if abs(f.normal.dot(dv)) < axis_dot and f.calc_area() > 0]
    sid = set(f.index for f in side)
    cosmax = math.cos(math.radians(25))
    patch = {}
    pid = 0
    for f0 in side:
        if f0.index in patch:
            continue
        st = [f0]; patch[f0.index] = pid
        while st:
            f = st.pop()
            for e in f.edges:
                for g in e.link_faces:
                    if g.index in sid and g.index not in patch and f.normal.dot(g.normal) > cosmax:
                        patch[g.index] = pid; st.append(g)
        pid += 1
    verts = [tuple(v.co) for v in bm.verts]
    polys = [[v.index for v in f.verts] for f in bm.faces]
    bvh = BVHTree.FromPolygons(verts, polys, epsilon=0.0)
    by_patch = collections.defaultdict(list)
    for f in side:
        by_patch[patch[f.index]].append(f)
    holes = []
    for p_, fs in by_patch.items():
        ids = set(f.index for f in fs)
        bedges = []
        for f in fs:
            for lp in f.loops:
                e = lp.edge
                if sum(1 for g in e.link_faces if g.index in ids) == 1:
                    bedges.append((lp.vert.index, lp.link_loop_next.vert.index))
        if not bedges:
            continue
        pts_all = np.array([verts[i] for e in bedges for i in e])
        ext_patch = float(np.linalg.norm(pts_all.max(0) - pts_all.min(0)))
        for L in _boundary_loops(bedges):
            P = np.array([verts[i] for i in L])
            ext = float(np.linalg.norm(P.max(0) - P.min(0)))
            if ext > max_hole_mm * MM or ext > 0.6 * ext_patch:
                continue
            c3 = P.mean(0)
            U, S, Vt = np.linalg.svd(P - c3)
            nrm = Vt[2]
            if S[2] / max(len(P), 1) ** 0.5 > 0.2 * MM:  # not planar: an outline wrapping a corner, not a hole
                continue
            # outward = the side the patch faces look to around the loop
            fn = np.mean([np.array(f.normal) for f in fs if any(v.index in set(L) for v in f.verts)] or [nrm], axis=0)
            if nrm @ fn < 0:
                nrm = -nrm
            u, w = geo.perp_basis(nrm)
            Q = np.c_[(P - c3) @ u, (P - c3) @ w]
            cc, r, dev = geo.fit_circle(Q)
            A = abs(geo.poly_area(Q))
            round_ = dev < 0.12 * r and abs(A - math.pi * r * r) < 0.2 * math.pi * r * r
            if round_:
                c3 = c3 + u * cc[0] + w * cc[1]
            elif A < min_poly_area_mm2 * MM * MM:
                continue
            # wall thickness beside the hole (rays through solid, 4 directions) and the hole's own depth
            rr = (r if round_ else math.sqrt(A / math.pi)) * 1.35 + 0.3 * MM
            th = []
            for ang in (0.3, 1.9, 3.5, 5.1):
                o = c3 + (u * math.cos(ang) + w * math.sin(ang)) * rr + nrm * 0.2 * MM
                h1 = bvh.ray_cast(Vector(o), Vector(-nrm), 0.2)
                if h1[0] is None or h1[3] > 0.6 * MM:
                    continue
                h2 = bvh.ray_cast(h1[0] - Vector(nrm) * 1e-6, Vector(-nrm), 0.2)
                if h2[0] is not None:
                    th.append(h1[3] + h2[3])
            if not th:
                continue
            t_wall = float(np.median(th)) - 0.2 * MM
            ha = bvh.ray_cast(Vector(c3 + nrm * 0.2 * MM), Vector(-nrm), 0.2)
            depth_axis = (ha[3] - 0.2 * MM) if ha[0] is not None else 1e9
            through = depth_axis > t_wall + 0.05 * MM
            depth = t_wall if through else depth_axis
            if depth < min_depth_mm * MM:
                continue  # engraving (logo / lettering): not kept (owner: no decals needed)
            h = {"c": c3, "n": nrm, "depth": depth, "through": through, "u": u, "w": w}
            if round_:
                h["r"] = r
            else:
                h["poly2d"] = build._canonical_loop(geo.douglas_peucker(build._canonical_loop(Q), 0.2 * MM, closed=True) - cc * 0)
            holes.append(h)
    bm.free()
    # a through hole shows as an inner loop on both faces of its wall: keep one per axis line. Round loops first: where
    # a threaded hole exits through an oblique face (24T bevel gear clamp cap) that loop is a non-round outline with a
    # tilted normal, and its cutter, driven along the tilted axis, widened the hole and pushed it into the ear's wall
    out = []
    for h in sorted(holes, key=lambda x: ("r" not in x, -x["depth"])):
        dup = False
        for k in out:
            if "r" in k and k["through"] and abs(h["n"] @ k["n"]) > 0.8:
                off = h["c"] - k["c"]
                perp = off - (off @ k["n"]) * k["n"]
                if np.linalg.norm(perp) < k["r"] and abs(off @ k["n"]) < k["depth"] + 0.5 * MM:
                    dup = True
                    break
            if abs(h["n"] @ k["n"]) > 0.98:
                off = h["c"] - k["c"]
                perp = off - (off @ k["n"]) * k["n"]
                same = ("r" in h and "r" in k and abs(h["r"] - k["r"]) < 0.3 * MM) or ("r" not in h and "r" not in k)
                if np.linalg.norm(perp) < 0.3 * MM and same and abs(off @ k["n"]) < k["depth"] + 0.5 * MM:
                    dup = True
                    break
        if not dup:
            out.append(h)
    return out


DEBUG_CYL = bool(os.environ.get("DEBUG_CYL"))


def cyl_holes(me, d, sides=default_sides, known=(), min_r_mm=1.0, max_r_mm=15.0, min_turn_deg=100.0, axis_dot=0.3):
    """Bores and scoops whose rim is NOT a flat loop, so side_holes misses them: concave cylindrical wall patches with
    their axis across d (a kit hub's clamp-screw counterbore opening on a rounded lobe; another kit hub's screw-head
    channel). The sections stacked along d fill them solid. One cylinder per patch from its closed end (a floor seen by
    rays near the wall) to its open end; rejected if more than 3% of the points inside it are inside the CAD solid."""
    bm = bmesh.new(); bm.from_mesh(me)
    bm.faces.ensure_lookup_table(); bm.normal_update()
    dv = np.asarray(d, float)
    cosmax = math.cos(math.radians(25))
    patch, pid = {}, 0
    for f0 in bm.faces:
        if f0.index in patch or f0.calc_area() <= 0:
            continue
        st = [f0]; patch[f0.index] = pid
        while st:
            f = st.pop()
            for e in f.edges:
                for g in e.link_faces:
                    if g.index not in patch and g.calc_area() > 0 and f.normal.dot(g.normal) > cosmax:
                        patch[g.index] = pid; st.append(g)
        pid += 1
    by = collections.defaultdict(list)
    for f in bm.faces:
        if f.index in patch:
            by[patch[f.index]].append(f)
    verts = [tuple(v.co) for v in bm.verts]
    polys = [[v.index for v in f.verts] for f in bm.faces]
    bvh = BVHTree.FromPolygons(verts, polys, epsilon=0.0)

    def inside(p):  # ray parity, majority of 3 directions (a nearest-face test misread points beside a clamp slit)
        votes = 0
        for dr in ((0.577, 0.577, 0.577), (-0.267, 0.802, -0.535), (0.707, -0.707, 0.1)):
            o, dv_, n = Vector(p), Vector(dr).normalized(), 0
            for _ in range(64):
                h = bvh.ray_cast(o, dv_, 1.0)
                if h[0] is None:
                    break
                n += 1; o = h[0] + dv_ * 1e-6
            votes += n % 2
        return votes >= 2

    out = []
    for fs in by.values():
        if len(fs) < 8:
            continue
        N = np.array([np.array(f.normal) for f in fs]); A = np.array([f.calc_area() for f in fs])
        if A.sum() < 2 * MM * MM:
            continue
        w_, V = np.linalg.eigh((N * A[:, None]).T @ N)
        a = V[:, 0]  # the normals of a cylinder are all perpendicular to its axis
        if w_[0] > 0.02 * w_.sum() or abs(a @ dv) > axis_dot:
            continue
        u, w = geo.perp_basis(a)
        P = np.array([np.array(f.calc_center_median()) for f in fs])
        Q = np.c_[P @ u, P @ w]
        cc, r, dev = geo.fit_circle(Q)
        if not (min_r_mm * MM <= r <= max_r_mm * MM) or dev > 0.06 * r:
            continue
        ang = np.arctan2(N @ w, N @ u)
        span = np.ptp(np.unwrap(np.sort(ang))) if len(ang) > 1 else 0
        if math.degrees(span) < min_turn_deg:
            continue
        c3 = u * cc[0] + w * cc[1]
        radial = (P - c3) - np.outer((P - c3) @ a, a)
        if np.mean(np.einsum("ij,ij->i", N, radial)) > 0:  # normals point away from the axis: a boss, not a bore
            continue
        t = P @ a
        t0, t1 = float(t.min()), float(t.max())
        if t1 - t0 < 0.5 * MM:
            continue
        line = lambda tt: c3 + a * tt

        def floor(tt, s):  # distance to a floor past this end (a fillet band can end the wall ~1 mm early), or None
            ds = []
            for k in range(4):
                th = k * math.pi / 2 + 0.4
                o = line(tt - s * 0.15 * MM) + (u * math.cos(th) + w * math.sin(th)) * 0.8 * r
                h = bvh.ray_cast(Vector(o), Vector(a * s), 1.5 * MM)
                if h[0] is not None and h[1].dot(Vector(a * s)) < 0:
                    ds.append(h[3] - 0.15 * MM)
            return min(ds) if ds else None
        f0_, f1_ = floor(t0, -1), floor(t1, 1)
        c0, c1 = f0_ is not None, f1_ is not None
        if c0:
            t0 -= f0_
        if c1:
            t1 += f1_
        if DEBUG_CYL:
            print("CYLDBG r %.2f axis %s c %s t %.2f..%.2f closed %s %s" % (r / MM, np.round(a, 2), np.round(c3 / MM, 2), t0 / MM, t1 / MM, c0, c1))
            for tt_, s_ in ((t0, -1), (t1, 1)):
                ds = []
                for k in range(4):
                    th = k * math.pi / 2 + 0.4
                    o = line(tt_ - s_ * 0.15 * MM) + (u * math.cos(th) + w * math.sin(th)) * 0.8 * r
                    h = bvh.ray_cast(Vector(o), Vector(a * s_), 5 * MM)
                    ds.append(None if h[0] is None else (round(h[3] / MM, 2), round(h[1].dot(Vector(a * s_)), 2)))
                print("CYLDBG   end", s_, ds)
        if c0 and c1:
            continue  # an internal void, not visible
        if any(abs(k["n"] @ a) > 0.95 and np.linalg.norm((k["c"] - line(0)) - ((k["c"] - line(0)) @ a) * a) < 0.5 * MM
               and abs(k.get("r", r) - r) < 0.5 * MM for k in list(known) + out):
            continue
        # cutter from the open end (cutter_mesh adds 0.5 mm outside) down to the closed end; open both ends = through
        cen, n_out = (line(t1), a) if not c1 else (line(t0), -a)
        depth = t1 - t0
        through = not (c0 or c1)
        # validate: points inside the cylinder must be outside the CAD solid
        bad = tot = 0
        for tt in np.linspace(0.1, 0.9, 5):
            for rr in (0.0, 0.5 * r, 0.85 * r):
                for th in np.linspace(0, 2 * math.pi, 8, endpoint=False):
                    p = cen - n_out * depth * tt + (u * math.cos(th) + w * math.sin(th)) * rr
                    tot += 1; bad += inside(p)
                    if DEBUG_CYL and inside(p):
                        print("CYLDBG     inside at tt %.1f rr %.2f th %.1f" % (tt, rr / MM, th))
        if DEBUG_CYL:
            print("CYLDBG   inside CAD %d / %d" % (bad, tot))
        if bad > 0.03 * tot:
            continue
        uu, ww = geo.perp_basis(n_out)
        # flats on the CAD radius (review 8: an inscribed octagon read as an undersized counterbore)
        rc = r / math.cos(math.pi / sides(r / MM))
        out.append({"c": cen, "n": n_out, "r": rc, "depth": depth, "through": through, "u": uu, "w": ww, "cyl": True})
    bm.free()
    return out


def cutter_mesh(holes, d, sides=default_sides, extra=0.5 * MM):
    """Closed prisms, one per hole: n-gon (round) or the simplified outline, from 0.5 mm outside the face to the hole
    bottom (blind) or 0.5 mm past the back face (through). Hexagons have one corner towards the stacking axis."""
    verts, faces = [], []
    for h in holes:
        c, n, u, w = h["c"], h["n"], h["u"], h["w"]
        dd = np.asarray(d, float) - (np.asarray(d, float) @ n) * n
        if np.linalg.norm(dd) > 1e-6:  # consistent hex orientation: a corner along the axis direction
            u = dd / np.linalg.norm(dd); w = np.cross(n, u)
        if "r" in h:
            k = sides(h["r"] / MM)
            ang = np.arange(k) * 2 * math.pi / k
            ring = [(h["r"] * math.cos(a), h["r"] * math.sin(a)) for a in ang]
        else:
            Q = h["poly2d"]
            ring = [(p[0], p[1]) for p in Q]
            u, w = h["u"], h["w"]
            if geo.poly_area(np.array(ring)) < 0:
                ring = ring[::-1]
        top = c + n * extra
        bot = c - n * (h["depth"] + (extra if h["through"] else 0.0))
        b = len(verts)
        k = len(ring)
        verts += [tuple(top + u * x + w * y) for x, y in ring]
        verts += [tuple(bot + u * x + w * y) for x, y in ring]
        faces.append(tuple(b + i for i in range(k)))
        faces.append(tuple(b + k + i for i in reversed(range(k))))
        for i in range(k):
            j = (i + 1) % k
            faces.append((b + i, b + k + i, b + k + j, b + j))
    me = bpy.data.meshes.new("__cutters")
    me.from_pydata(verts, [], faces)
    me.update()
    bm = bmesh.new(); bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me); bm.free()
    return me


BOOL_SOLVER = "EXACT"


def boolean(base_me, cut_me, name, solver=None, operation='DIFFERENCE'):
    """base minus cutters (or another boolean operation). EXACT with self-intersection (the stacked runs / walls overlap), or MANIFOLD (Blender 4.5+:
    robust on coplanar faces when every input piece is closed)."""
    sc = bpy.context.scene
    ob = bpy.data.objects.new("__base", base_me); sc.collection.objects.link(ob)
    oc = bpy.data.objects.new("__cut", cut_me); sc.collection.objects.link(oc)
    mod = ob.modifiers.new("cut", 'BOOLEAN')
    mod.operation = operation; mod.solver = solver or BOOL_SOLVER; mod.object = oc
    mod_solver_used = mod.solver
    if mod.solver == 'EXACT':
        mod.use_self = True; mod.use_hole_tolerant = True
    dg = bpy.context.evaluated_depsgraph_get()
    new = bpy.data.meshes.new_from_object(ob.evaluated_get(dg), preserve_all_data_layers=False, depsgraph=dg)
    new.name = name
    bpy.data.objects.remove(ob); bpy.data.objects.remove(oc)
    if solver is None and mod_solver_used == 'EXACT' and build.open_edges(new):
        # coplanar touching runs (a gearbox base's two levels) left open seams in the exact solver: Manifold is
        # watertight on closed inputs (it moved some faces up to ~1 mm on blocks, so it is the fallback only)
        alt = boolean(base_me, cut_me, name, solver='MANIFOLD', operation=operation)
        if build.open_edges(alt) < build.open_edges(new):
            bpy.data.meshes.remove(new)
            return alt
        bpy.data.meshes.remove(alt)
    return new


def join(meshes, name):
    bm = bmesh.new()
    for m in meshes:
        bm.from_mesh(m)
    out = bpy.data.meshes.new(name)
    bm.to_mesh(out); bm.free()
    return out


def remodel(name, me, axis="auto", tol_mm=0.3, sides=default_sides, cross=True, max_hole_mm=30.0, merge_mm=0.25,
            min_hole_r_mm=0.0, drop_holes_mm2=0.0, cyl=False, snap=False):
    k = axis if isinstance(axis, str) else None
    if k == "auto":
        k = auto_axis(me)
    if k == "bend":
        d = bend_axis(me); k = None
    else:
        d = AXES[k] if k else np.asarray(axis, float) / np.linalg.norm(axis)
    holes = side_holes(me, d, sides, max_hole_mm=max_hole_mm) if cross else []
    if cross and cyl:  # opt-in: bores / scoops with a non-flat rim (cyl_holes)
        holes += cyl_holes(me, d, sides, known=holes)
    holes = [h for h in holes if h.get("r", 1.0) >= min_hole_r_mm * MM]
    hh = geo.mesh_arrays(me)[0] @ d
    runs, lv = stack(name, me, d, tol_mm, sides, prefer=free_heights(holes, d, float(hh.min()), float(hh.max())),
                     merge_mm=merge_mm, drop_holes_mm2=drop_holes_mm2, snap=snap)
    if not runs:
        return None, {"error": "no sections"}
    base = join(runs, name + " [stack]")
    for m in runs:
        bpy.data.meshes.remove(m)
    cut = cutter_mesh(holes, d, sides) if holes else None
    if cut is None:  # still resolve the overlapping runs: a far-away dummy cutter
        cut = bpy.data.meshes.new("__cutters")
        bmc = bmesh.new(); bmesh.ops.create_cube(bmc, size=1e-4, matrix=Matrix.Translation((50, 50, 50))); bmc.to_mesh(cut); bmc.free()
    new = _finish(boolean(base, cut, name))
    bpy.data.meshes.remove(base); bpy.data.meshes.remove(cut)
    if build.open_edges(new):  # T-junction seams where stacked runs and cutters meet
        build.fix_t_junctions(new)
    info = {"method": "remodel (sections + cutters)", "axis": k or [round(float(x), 3) for x in d], "levels": len(lv),
            "runs": len(runs), "cross_holes": len(holes), "open_edges": build.open_edges(new)}
    e1, e2 = build.fit_error(me, new)
    info["err_mm"] = [round(e1, 2), round(e2, 2)]
    return new, info


def remodel_silhouette(name, me, axis, axis2, merge_mm=50.0, drop_holes_mm2=30.0, **kw):
    """remodel() along axis, intersected with the part's largest section along axis2 extruded through it (one run, holes
    under drop_holes_mm2 filled). Stacking along one axis squares off every outline that lies in the other planes: the
    24T bevel gear clamp cap's ear ends are rounded seen along the screw axis (172 -> 216 tris, review 6). Several runs
    along axis2 left a step where they meet, so axis2 is one run."""
    new, info = remodel(name, me, axis, **kw)
    if new is None:
        return new, info
    sil, _ = remodel(name + " [silhouette]", me, axis2, cross=False, merge_mm=merge_mm, drop_holes_mm2=drop_holes_mm2,
                     tol_mm=kw.get("tol_mm", 0.3))
    if sil is None:
        return new, info
    out = _finish(boolean(new, sil, name, operation='INTERSECT'))
    bpy.data.meshes.remove(new); bpy.data.meshes.remove(sil)
    if build.open_edges(out):
        build.fix_t_junctions(out)
    info.update(method="remodel (sections + cutters)", silhouette=axis2, open_edges=build.open_edges(out))
    e1, e2 = build.fit_error(me, out)
    info["err_mm"] = [round(e1, 2), round(e2, 2)]
    return out, info


# ------------------------------------------------------------------ thin-walled parts: union of wall slabs
def _normal_clusters(n, ar, cos_tol=math.cos(math.radians(1.0)), min_frac=0.02):
    """Sign-insensitive face-normal directions carrying at least min_frac of the area, biggest first."""
    order = np.argsort(-ar)
    left = np.ones(len(n), bool)
    out, A = [], ar.sum()
    for i in order:
        if not left[i]:
            continue
        c = n[i]
        m = left & (np.abs(n @ c) > cos_tol)
        if ar[m].sum() < min_frac * A:
            left[i] = False
            continue
        c = np.linalg.eigh((n[m] * ar[m, None]).T @ n[m])[1][:, -1]  # area-weighted mean axis
        m = left & (np.abs(n @ c) > cos_tol)
        out.append(c / np.linalg.norm(c))
        left &= ~m
    return out


def _flat_regions(co, tri, n, ar, c, cos_tol=math.cos(math.radians(1.0)), htol=2e-5):
    """Connected flat regions facing along +-c: [(height, side, area, in-plane lo, hi)]."""
    m = np.nonzero(np.abs(n @ c) > cos_tol)[0]
    if len(m) == 0:
        return []
    h = co[tri[m]].mean(1) @ c
    s = np.sign(n[m] @ c).astype(int)
    owner = collections.defaultdict(list)
    for k, t in enumerate(m):
        a, b, d_ = tri[t]
        for e in ((a, b), (b, d_), (d_, a)):
            owner[(min(e), max(e))].append(k)
    parent = list(range(len(m)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]; x = parent[x]
        return x
    for ks in owner.values():
        for k in ks[1:]:
            if s[k] == s[ks[0]] and abs(h[k] - h[ks[0]]) < htol:
                parent[find(k)] = find(ks[0])
    groups = collections.defaultdict(list)
    for k in range(len(m)):
        groups[find(k)].append(k)
    u, w = geo.perp_basis(c)
    out = []
    for ks in groups.values():
        ts = m[ks]
        P = co[tri[ts].ravel()]
        q = np.c_[P @ u, P @ w]
        out.append((float(np.average(h[ks], weights=ar[ts])), int(s[ks[0]]), float(ar[ts].sum()), q.min(0), q.max(0)))
    return out


def wall_slabs(me, max_t=6.0 * MM, min_area=40 * MM * MM):
    """(direction, h0, h1) for every wall: two opposite CONNECTED flat regions along one normal direction, at most
    max_t apart, each big (min_area) and overlapping the other in the plane. (Counting a whole row of hole-wall facets
    at one height as a region made 23 'walls' out of a tilted grid sheet.)"""
    co, tri, n, ar, cen = _tris(me)
    slabs = []
    for c in _normal_clusters(n, ar):
        big = [r for r in _flat_regions(co, tri, n, ar, c) if r[2] > min_area]
        for ha, sa, aa, loa, hia in big:
            if sa > 0:
                continue  # start from the face looking down the axis (-c): the wall lies above it
            best = None
            for hb, sb, ab, lob, hib in big:
                if sb < 0 or not (0.3 * MM < hb - ha <= max_t):
                    continue
                ov = np.minimum(hia, hib) - np.maximum(loa, lob)
                if (ov > 1 * MM).all() and (best is None or hb < best):
                    best = hb
            if best is not None and not any(abs(c @ c2) > 0.999 and abs(ha - a2) < 1e-5 and abs(best - b2) < 1e-5
                                            for c2, a2, b2 in slabs):
                slabs.append((c, ha, best))
    return slabs


SLAB_PAD = 5e-6


def slab_union(name, me, sides=default_sides, tol_mm=0.3):
    """Each wall = the part's section at the wall's mid-plane (its outline with every hole and slot) extruded through
    the wall; all walls united (exact boolean). Returns (mesh, info) or (None, info) when the part is not walls."""
    slabs = wall_slabs(me)
    if not slabs:
        return None, {"error": "no wall slabs"}
    seen, circ, parts = [], {}, []
    for i, (c, h0, h1) in enumerate(slabs):
        loops, u, w = build.section_loops(me, c, 0.5 * (h0 + h1))
        loops = _simplify(loops, tol_mm * MM, sides, seen, circ)
        if loops:  # 5 um proud on both faces: where walls overlap at a corner no two faces are coplanar (coplanar
            # overlaps left zero-length edges and open seams in the exact boolean on low-side channels)
            parts.append(build.extrude_loops(f"{name} wall{i}", loops, u, w, c, h0 - SLAB_PAD, h1 + SLAB_PAD))
    if not parts:
        return None, {"error": "empty wall sections"}
    base = join(parts, name + " [walls]")
    for m in parts:
        bpy.data.meshes.remove(m)
    cut = bpy.data.meshes.new("__cutters")
    bmc = bmesh.new(); bmesh.ops.create_cube(bmc, size=1e-4, matrix=Matrix.Translation((50, 50, 50))); bmc.to_mesh(cut); bmc.free()
    new = _finish(boolean(base, cut, name))
    bpy.data.meshes.remove(base); bpy.data.meshes.remove(cut)
    if build.open_edges(new):  # slivers where a wall's outline meets a corner: split T-junctions, fill planar seams
        build.fix_t_junctions(new)
    e1, e2 = build.fit_error(me, new)
    return new, {"method": "remodel (wall slabs)", "walls": len(slabs), "open_edges": build.open_edges(new),
                 "err_mm": [round(e1, 2), round(e2, 2)]}


def _finish(new):
    # the boolean result carries an empty material slot: drop it, or transfer_materials appends the CAD's material
    # behind it and every face keeps the empty slot (wheels rendered in the default grey)
    new.materials.clear()
    # no Merge by Distance here: the exact boolean output is watertight, and a 1e-7 merge tore a flap out of a
    # channel's bottom face (4 open edges). The coplanar dissolve is kept only when it stays watertight.
    bm = bmesh.new(); bm.from_mesh(new)
    bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(0.5), use_dissolve_boundaries=False, verts=bm.verts, edges=bm.edges)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    if sum(1 for e in bm.edges if len(e.link_faces) != 2) <= build.open_edges(new):
        bm.to_mesh(new)
    bm.free()
    new.update()
    drop_islands(new)
    for f in new.polygons:
        f.use_smooth = True
    new.set_sharp_from_angle(angle=math.radians(30))
    return new


def drop_islands(me, frac=0.02):
    """Delete disconnected islands under frac of the largest island's area: one CAD body is one solid, and a stray
    island is a stacking / boolean leftover (review 1: a 21 x 1 mm fin floating beside the ServoBlock)."""
    bm = bmesh.new(); bm.from_mesh(me); bm.faces.ensure_lookup_table()
    seen, isl = set(), []
    for f0 in bm.faces:
        if f0.index in seen:
            continue
        st, fs = [f0], [f0]; seen.add(f0.index)
        while st:
            f = st.pop()
            for e in f.edges:
                for g in e.link_faces:
                    if g.index not in seen:
                        seen.add(g.index); st.append(g); fs.append(g)
        isl.append((sum(f.calc_area() for f in fs), fs))
    n = 0
    if len(isl) > 1:
        big = max(a for a, _ in isl)
        kill = [f for a, fs in isl if a < frac * big for f in fs]
        n = sum(1 for a, _ in isl if a < frac * big)
        if kill:
            bmesh.ops.delete(bm, geom=kill, context='FACES')
            bm.to_mesh(me); me.update()
    bm.free()
    return n


# ------------------------------------------------------------------ bent grid sheets: develop, rebuild flat, bend back
def _profile_curve(P2, cell=3.0 * MM, tol=0.08 * MM):
    """Ordered centre line of a thin curved band of 2D points: 3 mm cell centroids, chained along the longest path of
    their nearest-neighbour graph, smoothed, Douglas-Peucker'd."""
    keys = np.floor(P2 / cell).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    C = np.zeros((len(uniq), 2)); np.add.at(C, inv.ravel(), P2); C /= np.bincount(inv.ravel())[:, None]
    # MST over cell centroids (Prim, O(n^2) on a few hundred cells)
    n = len(C)
    D = np.linalg.norm(C[:, None] - C[None], axis=2)
    intree = np.zeros(n, bool); intree[0] = True; best = D[0].copy(); par = np.zeros(n, int)
    adj = collections.defaultdict(list)
    for _ in range(n - 1):
        best[intree] = np.inf
        j = int(np.argmin(best)); intree[j] = True
        adj[j].append(par[j]); adj[par[j]].append(j)
        upd = D[j] < best; par[upd] = j; best = np.minimum(best, D[j])

    def far(a):
        dist = {a: 0.0}; st = [a]; prev = {a: None}
        while st:
            x = st.pop()
            for y in adj[x]:
                if y not in dist:
                    dist[y] = dist[x] + D[x, y]; prev[y] = x; st.append(y)
        b = max(dist, key=dist.get)
        return b, prev
    a, _ = far(0)
    b, prev = far(a)
    path = [b]
    while prev[path[-1]] is not None:
        path.append(prev[path[-1]])
    Q = C[path]
    for _ in range(3):  # smoothing (keep the ends): 3 mm cell centroids of both faces sit on the mid-surface
        if len(Q) >= 5:
            Q[1:-1] = (Q[:-2] + 2 * Q[1:-1] + Q[2:]) / 4
    # extend both ends to cover every point's projection
    return geo.douglas_peucker(Q, tol, closed=False)


def _project_polyline(P2, Q):
    """For 2D points: arc-length s of the closest point on polyline Q, signed offset z (left normal), segment index."""
    A, B = Q[:-1], Q[1:]
    AB = B - A; L = np.linalg.norm(AB, axis=1); T = AB / L[:, None]
    s0 = np.r_[0, np.cumsum(L)]
    out_s = np.empty(len(P2)); out_z = np.empty(len(P2))
    for i in range(0, len(P2), 4096):
        p = P2[i:i + 4096, None, :]
        t = np.clip(((p - A) * T).sum(2), 0, L)
        foot = A + t[..., None] * T
        d = np.linalg.norm(p - foot, axis=2)
        k = np.argmin(d, axis=1)
        r = np.arange(len(k))
        out_s[i:i + 4096] = s0[k] + t[r, k]
        nrm = np.c_[-T[k, 1], T[k, 0]]
        out_z[i:i + 4096] = ((P2[i:i + 4096] - foot[r, k]) * nrm).sum(1)
    return out_s, out_z, s0


def _bend_map(s, Q, s0):
    """(arc length) -> point and left normal on polyline Q (linear past the ends)."""
    k = np.clip(np.searchsorted(s0, s, side="right") - 1, 0, len(Q) - 2)
    T = Q[k + 1] - Q[k]; L = np.linalg.norm(T, axis=1); T = T / L[:, None]
    p = Q[k] + (s - s0[k])[:, None] * T
    return p, np.c_[-T[:, 1], T[:, 0]]


def sheet(name, me, sides=default_sides, tol_mm=0.3, bend_tol_mm=0.08):
    """Bent (or flat) grid sheet of constant thickness: the sheet is developed along its bend profile, its top face's
    outline and holes rebuilt flat (holes as n-gons, exact in the developed plane), extruded through the thickness,
    cut at every profile break line, and bent back. Any outline (edges oblique to the bend) and any hole layout."""
    co, tri, n, ar, cen = _tris(me)
    b = bend_axis(me)
    u, w = geo.perp_basis(b)
    P2 = np.c_[co @ u, co @ w]
    Q = _profile_curve(P2, tol=bend_tol_mm * MM)
    if len(Q) < 2:
        return None, {"error": "no profile"}
    # extend the profile 2 mm past both ends so every vertex projects inside it
    e0 = Q[0] + (Q[0] - Q[1]) / np.linalg.norm(Q[0] - Q[1]) * 2 * MM
    e1 = Q[-1] + (Q[-1] - Q[-2]) / np.linalg.norm(Q[-1] - Q[-2]) * 2 * MM
    Q = np.vstack([e0, Q, e1])
    s, z, s0 = _project_polyline(P2, Q)
    t = co @ b
    # top face = triangles whose normal follows the profile normal and lie on the + side
    cs, cz, _ = _project_polyline(np.c_[cen @ u, cen @ w], Q)
    _, Nc = _bend_map(cs, Q, s0)
    nn = (n @ u)[:, None] * 0 + np.c_[n @ u, n @ w]
    facing = (nn * Nc).sum(1)
    top = (facing > math.cos(math.radians(30))) & (cz > 0)
    bot = (facing < -math.cos(math.radians(30))) & (cz < 0)
    T_ = float(np.median(z[np.unique(tri[top])]) - np.median(z[np.unique(tri[bot])])) if top.any() and bot.any() else None
    if not T_ or T_ <= 0.2 * MM:
        return None, {"error": "no top/bottom faces"}
    zmid = float(np.median(z[np.unique(tri[top])]) + np.median(z[np.unique(tri[bot])])) / 2
    # boundary loops of the top region, in developed (s, t) coordinates
    owner = collections.Counter()
    tt = tri[top]
    for a_, b_, c_ in tt:
        for e in ((a_, b_), (b_, c_), (c_, a_)):
            owner[(min(e), max(e))] += 1
    bedges = []
    for a_, b_, c_ in tt:
        for e in ((a_, b_), (b_, c_), (c_, a_)):
            if owner[(min(e), max(e))] == 1:
                bedges.append(e)
    loops3 = _boundary_loops(bedges)
    loops2 = [np.c_[s[L], t[L]] for L in loops3 if len(L) >= 3]
    seen, circ = [], {}
    loops = _simplify(loops2, tol_mm * MM, sides, seen, circ)
    if not loops:
        return None, {"error": "empty developed outline"}
    flat = build.extrude_loops(name + " [flat]", loops, np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]),
                               zmid - T_ / 2, zmid + T_ / 2)
    # break lines across the sheet at every profile vertex inside the outline, so the bend follows the profile
    bm = bmesh.new(); bm.from_mesh(flat)
    smin, smax = min(L[:, 0].min() for L in loops), max(L[:, 0].max() for L in loops)
    for sk in s0[1:-1]:
        if smin + 0.05 * MM < sk < smax - 0.05 * MM:
            bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], dist=1e-7,
                                   plane_co=Vector((sk, 0, 0)), plane_no=Vector((1, 0, 0)))
    bmesh.ops.triangulate(bm, faces=[f for f in bm.faces if len(f.verts) > 4])
    V = np.array([v.co[:] for v in bm.verts])
    p, N = _bend_map(V[:, 0], Q, s0)
    W = np.outer(p[:, 0], u) + np.outer(p[:, 1], w) + np.outer(V[:, 1], b) + (np.outer(N[:, 0], u) + np.outer(N[:, 1], w)) * V[:, 2:3]
    for v, x in zip(bm.verts, W):
        v.co = Vector(x)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    new = bpy.data.meshes.new(name)
    bm.to_mesh(new); bm.free()
    bpy.data.meshes.remove(flat)
    new = _finish(new)
    e1, e2 = build.fit_error(me, new)
    return new, {"method": "remodel (developed sheet)", "thickness_mm": round(T_ / MM, 2), "profile_pts": len(Q) - 2,
                 "loops": len(loops), "open_edges": build.open_edges(new), "err_mm": [round(e1, 2), round(e2, 2)]}


# ------------------------------------------------------------------ lathe + what sticks out of it (gear with clamp ears)
def lathe_plus(name, me, lathe_me, d, c, out_mm=0.8, bin_mm=0.5, tol_mm=0.3, sides=default_sides, mode="section",
               hub_tris=300):
    """A lathe of a part that is round except for features that stick out of it (a gear with a clamping hub: the
    lathe makes the ears a round boss; the mating clamp cap keeps its ears). The heights where CAD surface points lie
    more than out_mm outside the lathe's radius get the part's own section there (max-area slice) extruded, united
    with the lathe. Returns (mesh, info) or (None, info) when nothing sticks out."""
    d = np.asarray(d, float); d /= np.linalg.norm(d); c = np.asarray(c, float)
    co, tri, _ = geo.mesh_arrays(me)
    pts, _ = geo.sample_surface(co, tri, 40000)
    rel = pts - c
    h = rel @ d
    r = np.linalg.norm(rel - np.outer(h, d), axis=1)
    lco, ltri, _ = geo.mesh_arrays(lathe_me)
    lv, _ = geo.sample_surface(lco, ltri, 40000)  # surface samples: the lathe's few profile vertices leave bins empty
    lrel = lv - c
    lh = lrel @ d
    lr = np.linalg.norm(lrel - np.outer(lh, d), axis=1)
    lo, hi = float(h.min()), float(h.max())
    edges = np.arange(lo, hi + bin_mm * MM, bin_mm * MM)
    out_bins = []
    for a, b in zip(edges[:-1], edges[1:]):
        m = (h >= a) & (h < b)
        lm = (lh >= a) & (lh < b)
        if not m.any():
            continue
        if r[m].max() > (lr[lm].max() if lm.any() else 0.0) + out_mm * MM:
            out_bins.append((a, b))
    if not out_bins:
        return None, {"error": "nothing sticks out of the lathe"}
    # contiguous ranges of sticking-out bins
    ranges = []
    for a, b in out_bins:
        if ranges and a - ranges[-1][1] < 1e-7:
            ranges[-1][1] = b
        else:
            ranges.append([a, b])
    # a range that stops less than 1 mm short of the part's end runs to the end: a thin round lathe slice left below the
    # clamp covered the mating cap's half of the end face (21.8 mm2 coplanar, same-facing: z-fighting)
    END_GAP = 5e-5  # the part's own section stops 0.05 mm short of an end it runs to: the mating cap's end face lies
    # in that plane and interlocks under the section (4-7 mm2 same-facing coplanar on the 24T gear otherwise)
    cut_ranges = []
    for rg in ranges:
        c0, c1 = rg[0], rg[1]
        if rg[0] - lo < 1.0 * MM:
            c0 = lo - 1.0 * MM; rg[0] = lo + END_GAP
        if hi - rg[1] < 1.0 * MM:
            c1 = hi + 1.0 * MM; rg[1] = hi - END_GAP
        cut_ranges.append((c0, c1))
    # over the sticking-out heights the lathe is cut away and the part's own section takes its place (round boss there
    # would overlap the mating half: review 2 measured 4-24 mm2 of same-facing faces within 0.05 mm of a clamp cap)
    R = float(np.linalg.norm(co - c, axis=1).max()) * 1.5
    cutv, cutf = [], []
    for a, b in cut_ranges:
        u0, w0 = geo.perp_basis(d)
        k0 = len(cutv); n_ = 48
        for z in (float(c @ d) + a, float(c @ d) + b):
            for i in range(n_):
                t = 2 * math.pi * i / n_
                cutv.append(tuple(c - d * float(c @ d) + d * z + R * (u0 * math.cos(t) + w0 * math.sin(t))))
        cutf.append(tuple(k0 + i for i in range(n_))[::-1]); cutf.append(tuple(k0 + n_ + i for i in range(n_)))
        for i in range(n_):
            j = (i + 1) % n_
            cutf.append((k0 + i, k0 + j, k0 + n_ + j, k0 + n_ + i))
    slab = bpy.data.meshes.new("__slab"); slab.from_pydata(cutv, [], cutf); slab.update()
    bms = bmesh.new(); bms.from_mesh(slab); bmesh.ops.recalc_face_normals(bms, faces=bms.faces); bms.to_mesh(slab); bms.free()
    lcut = boolean(lathe_me.copy(), slab, name + " [lathe cut]")
    bpy.data.meshes.remove(slab)
    parts = [lcut]
    seen, circ = [], {}
    if mode == "decimate":
        # (review 5) the clamp half is not prismatic (rounded hub top, ears at part heights): the CAD's own geometry over
        # the range, cut off, its small holes plugged (the clamp screws fill them), and Collapse-decimated
        hc = float(c @ d)
        for a, b in ranges:
            hub = me.copy()
            bm = bmesh.new(); bm.from_mesh(hub)
            if b < hi - 2 * END_GAP:
                r_ = bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], dist=1e-7,
                                            plane_co=Vector(d * (hc + b + 2e-5)), plane_no=Vector(d), clear_outer=True)
            if a > lo + 2 * END_GAP:
                bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], dist=1e-7,
                                       plane_co=Vector(d * (hc + a - 2e-5)), plane_no=Vector(d), clear_inner=True)
            bnd = [e for e in bm.edges if len(e.link_faces) == 1]
            if bnd:
                bmesh.ops.holes_fill(bm, edges=bnd, sides=0)
            bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
            bm.to_mesh(hub); bm.free(); hub.update()
            build.prep_for_decimate(hub, plug_max=8e-3)
            t0 = build.tri_count(hub)
            ob = bpy.data.objects.new("__hub", hub); bpy.context.scene.collection.objects.link(ob)
            m = ob.modifiers.new("c", 'DECIMATE'); m.ratio = min(1.0, hub_tris / max(t0, 1))
            dg = bpy.context.evaluated_depsgraph_get()
            hd = bpy.data.meshes.new_from_object(ob.evaluated_get(dg), depsgraph=dg)
            bpy.data.objects.remove(ob); bpy.data.meshes.remove(hub)
            build.fix_inverted_shells(hd)
            parts.append(hd)
        ranges_iter = []
    else:
        ranges_iter = ranges
    # the ears: ONE prism per range from a slice no cross hole cuts (stacked slices left stair steps and 4-sided screw
    # openings, review 3), then their screw holes cut by n-gon cutters like the other remodels
    holes = side_holes(me, d, sides) if ranges_iter else []
    hc = float(c @ d)
    for a, b in ranges_iter:
        free = [x for x in free_heights(holes, d, hc + a, hc + b) if hc + a + 0.3 * MM < x < hc + b - 0.3 * MM]
        cands = []
        for x in (free[:6] or [hc + a + f * (b - a) for f in (0.3, 0.5, 0.7)]):
            loops, u, w = build.section_loops(me, d, float(x))
            if loops:
                cands.append((region_area(loops), loops, u, w))
        if not cands:
            continue
        best = max(cands, key=lambda t: t[0])
        loops = _simplify(best[1], tol_mm * MM, sides, seen, circ)
        if snap and runs and loops:  # opt-in: share the previous run's outline where it is within reach (_snap_to); two
            # outlines simplified at tol each can differ by 2 tol (1 tol left a tapered sliver beside the Hyper Hub's slit)
            loops = _snap_to(loops, runs[-1][0], (snap if isinstance(snap, (int, float)) and snap is not True else 2.0) * tol_mm * MM)
        if loops:  # 20 um into the lathe at an inner end: overlapping volumes, no coplanar caps
            e0 = 2e-5 if a > lo + 2 * END_GAP else 0.0
            e1 = 2e-5 if b < hi - 2 * END_GAP else 0.0
            ear = build.extrude_loops(name + " [ears]", loops, best[2], best[3], d, hc + a - e0, hc + b + e1)
            hs = [h for h in holes if hc + a - 0.5 * MM <= float(h["c"] @ d) <= hc + b + 0.5 * MM]
            if hs:
                cut = cutter_mesh(hs, d, sides)
                ear2 = boolean(ear, cut, name + " [ears cut]")
                bpy.data.meshes.remove(ear); bpy.data.meshes.remove(cut)
                ear = ear2
            parts.append(ear)
    base = join(parts, name + " [lathe+]")
    for m in parts:
        bpy.data.meshes.remove(m)
    cut = bpy.data.meshes.new("__cutters")
    bmc = bmesh.new(); bmesh.ops.create_cube(bmc, size=1e-4, matrix=Matrix.Translation((50, 50, 50))); bmc.to_mesh(cut); bmc.free()
    new = _finish(boolean(base, cut, name))
    bpy.data.meshes.remove(base); bpy.data.meshes.remove(cut)
    if build.open_edges(new):
        build.fix_t_junctions(new)
    e1, e2 = build.fit_error(me, new)
    return new, {"method": "cylinder rebuild", "note": "lathe + sticking-out features (remodel.lathe_plus)",
                 "ranges_mm": [[round(a / MM, 2), round(b / MM, 2)] for a, b in ranges],
                 "open_edges": build.open_edges(new), "err_mm": [round(e1, 2), round(e2, 2)]}


# ------------------------------------------------------------------ in-place hole rebuild (any surface, flat or curved)
def _circle3(P):
    c = P.mean(0)
    U, S, Vt = np.linalg.svd(P - c)
    nrm = Vt[2]
    u, w = Vt[0], Vt[1]
    Q = np.c_[(P - c) @ u, (P - c) @ w]
    cc, r, dev = geo.fit_circle(Q)
    return c + u * cc[0] + w * cc[1], nrm, r, dev, S[2] / max(1, len(P)) ** 0.5


STATS = collections.Counter()
KEPT = set()


def reduce_holes(me, sides=default_sides, sharp_deg=45.0, max_r=8.0 * MM):
    """Every round hole of the part (through or blind; a smooth wall band bounded by two circular rims, threads and
    chamfers included) rebuilt in place as a k-sided hole (k = sides(r)): k rim vertices per rim snapped onto the exact
    n-gon, the other rim vertices dissolved into the surrounding surface, the wall band replaced by k quads. The rest
    of the surface (its curvature, outline, other features) is untouched. Returns the number of holes rebuilt."""
    bm = bmesh.new(); bm.from_mesh(me)
    bm.faces.ensure_lookup_table()
    lim = math.radians(sharp_deg)
    sharp = set(e for e in bm.edges if (not e.is_manifold) or e.calc_face_angle(0.0) > lim)
    seen, holes = set(), []
    for f0 in bm.faces:
        if f0.index in seen:
            continue
        patch, st = [f0], [f0]; seen.add(f0.index)
        while st:
            f = st.pop()
            for e in f.edges:
                if e in sharp:
                    continue
                for g in e.link_faces:
                    if g.index not in seen:
                        seen.add(g.index); patch.append(g); st.append(g)
                        if len(patch) > 600:
                            break
        if len(patch) > 600:
            continue
        ps = set(patch)
        bedges = [e for f in patch for e in f.edges if sum(1 for g in e.link_faces if g in ps) == 1]
        pts = np.array([v.co[:] for e in bedges for v in e.verts])
        if len(pts) == 0 or np.linalg.norm(pts.max(0) - pts.min(0)) > 2.6 * max_r + 3 * MM:
            continue
        # two closed rim loops (ordered vertex lists)
        adj = collections.defaultdict(list)
        for e in bedges:
            a, b = e.verts; adj[a].append(b); adj[b].append(a)
        if any(len(v) != 2 for v in adj.values()):
            continue
        loops, used = [], set()
        for v0 in adj:
            if v0 in used:
                continue
            L, prev, cur = [v0], None, v0; used.add(v0)
            while True:
                nx = [x for x in adj[cur] if x is not prev]
                nxt = nx[0] if nx[0] not in used or len(nx) == 1 else nx[1]
                if nxt is v0 or nxt in used:
                    break
                L.append(nxt); used.add(nxt); prev, cur = cur, nxt
            loops.append(L)
        if len(loops) != 2 or min(len(L) for L in loops) < 6:
            continue
        rims = []
        for L in loops:
            P = np.array([v.co[:] for v in L])
            c, nrm, r, dev, flat = _circle3(P)
            if r > max_r or dev > 0.15 * r or flat > 0.1 * r:
                break
            rims.append((L, c, nrm, r))
        if len(rims) != 2:
            continue
        (La, ca, na, ra), (Lb, cb, nb, rb) = rims
        ax = cb - ca
        if np.linalg.norm(ax) < 1e-5 or abs(na @ nb) < 0.97:
            continue
        ax /= np.linalg.norm(ax)
        if abs(ax @ na) < 0.97:  # rims must be coaxial (a wall band, not some other strip)
            continue
        holes.append((patch, rims, ax))
    # rebuild (patches are disjoint; rim verts are shared only with the surrounding surface)
    n_done = 0
    ref = None
    for patch, rims, ax in holes:
        u, w = geo.perp_basis(ax)
        r = float(np.mean([rims[0][3], rims[1][3]]))
        k = sides(r / MM)
        th = np.arange(k) * 2 * math.pi / k
        kept = []
        for L, c, nrm, r_ in rims:
            P = np.array([v.co[:] for v in L])
            ang = np.arctan2((P - c) @ w, (P - c) @ u)
            pick = []
            for t_ in th:
                dlt = np.abs((ang - t_ + math.pi) % (2 * math.pi) - math.pi)
                for i in np.argsort(dlt):
                    if L[i] not in pick:
                        pick.append(L[i]); break
            kept.append(pick)
        if any(len(set(p)) != k for p in kept):
            continue
        bmesh.ops.delete(bm, geom=list(set(patch)), context='FACES_ONLY')
        for (L, c, nrm, r_), pick in zip(rims, kept):
            drop = [v for v in L if v not in pick and v.is_valid]
            bmesh.ops.dissolve_verts(bm, verts=drop, use_face_split=False, use_boundary_tear=False)
            STATS["invalid_pick"] += sum(1 for v in pick if not v.is_valid)
            for t_, v in zip(th, pick):
                v.co = Vector(c + r_ * (u * math.cos(t_) + w * math.sin(t_)))
        A, B = kept
        KEPT.update(A); KEPT.update(B)
        for j in range(k):
            a0, a1, b0, b1 = A[j], A[(j + 1) % k], B[j], B[(j + 1) % k]
            try:
                bm.faces.new((a0, a1, b1, b0))
            except ValueError as ex:
                STATS["wall_fail"] += 1
        n_done += 1
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context='VERTS')
    # dissolving rim vertices can drop a surface face next to a rim (small closed notches, 3-10 vertices); the holes
    # themselves are walled, so every boundary loop left is such a notch: fill it
    bnd = [e for e in bm.edges if len(e.link_faces) == 1]
    if bnd:
        STATS["notch_edges"] += len(bnd)
        bmesh.ops.holes_fill(bm, edges=bnd, sides=24)
    bmesh.ops.triangulate(bm, faces=[f for f in bm.faces if len(f.verts) > 4], quad_method='BEAUTY', ngon_method='BEAUTY')
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me); bm.free(); me.update()
    return n_done


if __name__ == "__main__":
    import sys, os, time
    a = sys.argv[sys.argv.index("--") + 1:]
    t = time.time()
    o = bpy.data.objects[a[0]]
    me = o.data.copy()
    build.weld(me, 1e-6)
    if a[1] == "slabs":
        new, info = slab_union(o.name + " [remodel]", me)
    else:
        new, info = remodel(o.name + " [remodel]", me, a[1])
    build.transfer_materials(me, new)
    print("REMODEL", o.name, info, "tris", build.tri_count(o.data), "->", build.tri_count(new), "in", round(time.time() - t, 1), "s", flush=True)
    for x in list(bpy.data.objects):
        if x is not o:
            bpy.data.objects.remove(x)
    o.matrix_world = Matrix.Identity(4)
    r = bpy.data.objects.new(o.name + " [remodel]", new)
    bpy.context.scene.collection.objects.link(r)
    bpy.ops.wm.save_as_mainfile(filepath=a[2], compress=True)
