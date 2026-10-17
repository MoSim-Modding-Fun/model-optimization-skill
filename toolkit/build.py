# Mesh builders: revolve (lathe) remodel, 2.5D extrusion remodel, procedural hole tubes, helpers.
import bpy, bmesh, math, collections
import numpy as np
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
from mathutils.geometry import tessellate_polygon
import geo

MM = 0.001
# closed prisms that meet end to end (hex body + journal stubs, sectioned shafts) overlap by this much, so the
# narrower one's cap lies inside its neighbour instead of in the neighbour's end-face plane (z-fighting)
JUNCTION_OVERLAP = 0.1 * MM


# ---------------------------------------------------------------- generic

def new_mesh_from(name, verts, faces, smooth_faces=None, sharp_edges=None):
    """faces: list of vertex index tuples. smooth_faces: set of face indices to shade smooth.
    sharp_edges: set of frozenset({a,b}) vertex pairs marked sharp."""
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(v) for v in verts], [], [tuple(f) for f in faces])
    me.validate(clean_customdata=False)
    me.update()
    for p in me.polygons:
        p.use_smooth = bool(smooth_faces and p.index in smooth_faces)
    if sharp_edges:
        for e in me.edges:
            if frozenset(e.vertices) in sharp_edges:
                e.use_edge_sharp = True
    return me


def fix_normals(me):
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    me.update()


def transfer_materials(src_me, dst_me):
    """Copy material slots and pick each new face's material from the nearest original face.
    A face whose centre lies deep inside the solid (> 0.5 mm from any CAD face: a lathe cap fan running from the
    axis out to a shaft's collar rim) takes the nearest CAD face within a further 1.5 mm that faces the same way
    (normals within ~25 deg); the bare centre pick there alternated shaft / screw colours triangle by triangle. Where
    faces of different materials tie for nearest (within 0.02 mm), the one facing the same way as the new face wins."""
    for m in src_me.materials:
        dst_me.materials.append(m)
    if len(src_me.materials) <= 1:
        return
    co, tri, mat = geo.mesh_arrays(src_me)
    bvh = BVHTree.FromPolygons([Vector(c) for c in co], [tuple(t) for t in tri])
    for p in dst_me.polygons:
        c = p.center
        loc, nrm, idx, dist = bvh.find_nearest(c)
        if idx is None:
            continue
        m = int(mat[idx])
        near = bvh.find_nearest_range(c, dist + (1.5e-3 if dist > 5e-4 else 2e-5))
        if dist > 5e-4:
            same = [h for h in near if h[1].dot(p.normal) > 0.9]
            if same:
                m = int(mat[min(same, key=lambda h: h[3])[2]])
        elif len({int(mat[h[2]]) for h in near}) > 1:
            m = int(mat[max(near, key=lambda h: h[1].dot(p.normal))[2]])
        p.material_index = m


def fit_error(src_me, dst_me, n=6000, q=95):
    """Two-sided surface distance (mm): q-th percentile original->new, new->original."""
    co1, tri1, _ = geo.mesh_arrays(src_me)
    co2, tri2, _ = geo.mesh_arrays(dst_me)
    if len(tri2) == 0:
        return 999.0, 999.0
    b1 = BVHTree.FromPolygons([Vector(c) for c in co1], [tuple(t) for t in tri1])
    b2 = BVHTree.FromPolygons([Vector(c) for c in co2], [tuple(t) for t in tri2])
    p1, _ = geo.sample_surface(co1, tri1, n)
    p2, _ = geo.sample_surface(co2, tri2, n)
    d12 = np.array([b2.find_nearest(Vector(p))[3] or 0 for p in p1])
    d21 = np.array([b1.find_nearest(Vector(p))[3] or 0 for p in p2])
    return float(np.percentile(d12, q) / MM), float(np.percentile(d21, q) / MM)


def open_edges(me):
    """Edges with fewer than two faces (see-through seams once Unity culls backfaces)."""
    bm = bmesh.new()
    bm.from_mesh(me)
    n = sum(1 for e in bm.edges if len(e.link_faces) < 2)
    bm.free()
    return n


def tri_count(me):
    return sum(len(p.vertices) - 2 for p in me.polygons)


# ---------------------------------------------------------------- revolve

def revolve_profile(co, tri, d, centre, nsec=24, bin_mm=0.15, toothed=False, bore_max=False):
    """Outer/inner radius stair profiles along axis d. Returns h edges, R_out, R_in (metres)."""
    pts, _ = geo.sample_surface(co, tri, 60000)
    pts = np.vstack([pts, co])
    u, w = geo.perp_basis(d)
    rel = pts - centre
    h = rel @ d
    ru, rw = rel @ u, rel @ w
    r = np.hypot(ru, rw)
    th = np.arctan2(rw, ru)
    h0, h1 = h.min(), h.max()
    nb = int(np.clip((h1 - h0) / (bin_mm * MM), 8, 800))
    b = np.clip(((h - h0) / max(h1 - h0, 1e-12) * nb).astype(int), 0, nb - 1)
    s = ((th + math.pi) / (2 * math.pi) * nsec).astype(int) % nsec
    key = b * nsec + s
    smax = np.full(nb * nsec, np.nan)
    smin = np.full(nb * nsec, np.nan)
    order = np.argsort(key)
    ks, rs = key[order], r[order]
    uk, start = np.unique(ks, return_index=True)
    mx = np.maximum.reduceat(rs, start)
    mn = np.minimum.reduceat(rs, start)
    smax[uk] = mx
    smin[uk] = mn
    smax = smax.reshape(nb, nsec)
    smin = smin.reshape(nb, nsec)
    with np.errstate(all="ignore"):
        if (h1 - h0) > 6 * np.nanmax(smax):
            # slender parts (bolts, threaded rods, pins): the major diameter; median/mean land near the thread root
            # and the bolt looks ~25% too thin
            R_out = np.nanpercentile(smax, 90, axis=1)
        elif toothed:
            # between tip and root: mean of sector maxima
            R_out = np.nanmean(smax, axis=1)
        else:
            R_out = np.nanmedian(smax, axis=1)
        if bore_max:
            # largest sector minimum (hex corners), but a sector with no bore-wall samples reports the next surface
            # out (a spoke window: a 16 mm swerve gear bore came out 48 mm) -> ignore sectors far above the median.
            # On a long thin-walled tube most cells hold only outer-surface samples, so "far above" must also
            # mean "near the outer surface": 1.35 x median let the bore reach the outer radius (inner = outer, and the
            # final weld collapsed the roller into an open shell). Hex bores are unaffected (1.35 x med is lower there).
            med = np.nanmedian(smin, axis=1)
            lim = np.minimum(1.35 * med, med + 0.7 * (np.nanmedian(smax, axis=1) - med))
            R_in = np.nanmax(np.where(smin <= lim[:, None], smin, np.nan), axis=1)
        else:
            R_in = np.nanmedian(smin, axis=1)
    # fill empty bins from neighbours
    for arr in (R_out, R_in):
        idx = np.arange(nb)
        ok = ~np.isnan(arr)
        if ok.sum() == 0:
            arr[:] = 0
        else:
            arr[~ok] = np.interp(idx[~ok], idx[ok], arr[ok])
    edges = h0 + (h1 - h0) * np.arange(nb + 1) / nb
    return edges, R_out, R_in


def _median_filter(a, k):
    if len(a) < k:
        return a
    p = k // 2
    pad = np.pad(a, p, mode="edge")
    return np.median(np.lib.stride_tricks.sliding_window_view(pad, k), axis=1)


def stair_to_polyline(edges, R):
    pts = []
    for i in range(len(R)):
        pts.append((edges[i], R[i]))
        pts.append((edges[i + 1], R[i]))
    return np.array(pts)


# remodel lathes: belt/cord grooves and other notches narrower than this become one plain cylinder (owner:
# "you dont need grooves. one cylinder can replace all the grooves"); steps, flanges, hubs and cones stay
REMODEL_WINDOW_MM = 6.0


def split_merged_hole(H, jump=1e-3, n_out=24):
    """Round holes the tube analyser traced as ONE loop (joined by bridge segments from a CAD sliver: two 5 mm
    holes 12.7 mm apart became a 17.7 mm "slot" the wall triangulation could not cut). Pieces between jumps longer than
    `jump` must all be circle arcs (>= 12 points, residual < 3% of r); arcs sharing a centre are one circle (the middle
    hole of a chain of three is visited twice); every circle must span > 270 deg (a real obround's pieces are its two
    ~180 deg ends, with different centres). Returns a list of hole polygons (n_out-gons), or None (real slots stay)."""
    P = np.asarray(H, float)
    if len(P) < 24:
        return None
    seg = np.linalg.norm(np.roll(P, -1, 0) - P, axis=1)
    cut = np.where(seg > jump)[0]
    if len(cut) < 2:
        return None
    arcs = []
    for a, b in zip(cut, np.roll(cut, -1)):
        piece = P[a + 1:b + 1] if b > a else np.vstack([P[a + 1:], P[:b + 1]])
        if len(piece) < 3:
            continue  # bridge end points
        if len(piece) < 12:
            return None
        x, y = piece[:, 0], piece[:, 1]
        A = np.c_[2 * x, 2 * y, np.ones(len(x))]
        cx, cy, k = np.linalg.lstsq(A, x * x + y * y, rcond=None)[0]
        r = math.sqrt(max(k + cx * cx + cy * cy, 0.0))
        if r <= 0 or np.abs(np.hypot(x - cx, y - cy) - r).max() > 0.03 * r:
            return None
        arcs.append((cx, cy, r, piece))
    groups = []
    for cx, cy, r, piece in arcs:
        for g in groups:
            if math.hypot(cx - g[0], cy - g[1]) < 0.25 * r and abs(r - g[2]) < 0.1 * r:
                g[3].append(piece)
                break
        else:
            groups.append([cx, cy, r, [piece]])
    circles = []
    for cx, cy, r, pieces in groups:
        pts = np.vstack(pieces)
        th = np.sort(np.arctan2(pts[:, 1] - cy, pts[:, 0] - cx))
        if 2 * math.pi - np.diff(np.r_[th, th[0] + 2 * math.pi]).max() < math.radians(270):
            return None
        ang = np.arange(n_out) * 2 * math.pi / n_out
        circles.append(np.c_[cx + r * np.cos(ang), cy + r * np.sin(ang)])
    return circles if len(circles) >= 2 else None


def hole_is_engraving(H, max_verts=400):
    """A tube "hole" that is really an engraved script or logo: its traced outline has hundreds of points and is
    not round. One intake tube: two script loops of 780-793 points (compactness 0.48) next to round holes of 144 points;
    real slots are compact ~0.55 too, so the point count is what separates them."""
    P = np.asarray(H, float)[:, :2]
    if len(P) <= max_verts:
        return False
    per = float(np.sum(np.linalg.norm(np.roll(P, -1, 0) - P, axis=1)))
    return per > 0 and 4 * math.pi * abs(geo.poly_area(P)) / per ** 2 < 0.7


def _closing(a, k):
    """Grey-scale closing over k samples (max filter, then min filter): fills valleys narrower than k, keeps steps.
    A filled run that touches either end of the profile is put back: it is a step (a hub's insert, a flange's rim in
    the radial profile), not a groove, and filling it put a hub's surface onto its roller tube (z-fighting)."""
    p = k // 2
    if p < 1 or len(a) < 3:
        return a
    a = np.asarray(a, float)
    sw = np.lib.stride_tricks.sliding_window_view
    pad = lambda x, v: np.concatenate([np.full(p, v), x, np.full(p, v)])
    out = np.min(sw(pad(np.max(sw(pad(a, -np.inf), 2 * p + 1), axis=1), np.inf), 2 * p + 1), axis=1)
    filled = out > a + 1e-9
    # (a poly-V pulley) the put-back stops where the profile starts falling again going inward: a lip
    # rib at the end and the groove after it are one filled run, and only the rib is the step
    for idx in (range(len(a)), range(len(a) - 1, -1, -1)):
        top = -np.inf
        for i in idx:
            if not filled[i] or a[i] < top - 1e-4:
                break
            top = max(top, a[i])
            out[i] = a[i]
    return out


def _snap_profile(pts, snap_h, snap_r):
    """Remodel look: near-vertical profile segments become exact steps (flat faces), near-flat ones exact cylinders;
    collinear points dropped. The first and last points keep their heights (the part's length)."""
    pts = [list(map(float, p)) for p in pts]
    if len(pts) < 3:
        return np.array(pts)
    out = [pts[0]]
    for i, (h, r) in enumerate(pts[1:], 1):
        h0, r0 = out[-1]
        dh, dr = abs(h - h0), abs(r - r0)
        if dh < snap_h and dr >= dh and i < len(pts) - 1:
            h = h0
        elif dr < snap_r and dh >= dr:
            r = r0
        if abs(h - h0) < 1e-7 and abs(r - r0) < 1e-7:
            continue
        out.append([h, r])
    clean = [out[0]]
    for i in range(1, len(out) - 1):
        a, b, c = np.array(clean[-1]), np.array(out[i]), np.array(out[i + 1])
        if abs((b - a)[0] * (c - b)[1] - (b - a)[1] * (c - b)[0]) < 1e-12 and (b - a) @ (c - b) > 0:
            continue
        clean.append(out[i])
    clean.append(out[-1])
    return np.array(clean)


def build_revolve(name, src_me, d, centre, keep_bore, toothed=False, nseg=None, tol_mm=None, mat_rings=True,
                  bore_max=False, inset=(0.0, 0.0), radial=None, bore_steps=False, remodel=False):
    """inset: (start, end) in metres to pull the end faces in, where they lie flush on another part (z-fighting).
    bore_steps (with bore_max): keep the bore's own steps (counterbores), floored at the flush section's radius.
    remodel (cfg.LATHE_REMODEL): grooves and notches narrower than REMODEL_WINDOW_MM closed into plain cylinders
    and the profile snapped to exact cylinders and flat steps, so the part reads as a clean turned remodel."""
    co, tri, _ = geo.mesh_arrays(src_me)
    edges, R_out, R_in = revolve_profile(co, tri, d, centre, toothed=toothed, bore_max=bore_max)
    R_out = _median_filter(R_out, 5)
    R_in = _median_filter(R_in, 5)
    if remodel:
        # window capped at a third of the part: wider than the part, the closing flattens a dome or a cone into a puck
        kw = min(int(round(REMODEL_WINDOW_MM * MM / max((edges[-1] - edges[0]) / len(R_out), 1e-9))), len(R_out) // 3)
        R_out = _closing(R_out, kw)          # outer grooves (valleys) filled
        R_in = -_closing(-R_in, kw)          # bore grooves (peaks of the inner radius) removed
    if bore_max:
        # open bore around a flush shaft: one plain bore through the *narrowest* section, at its largest radius
        # (the hex corners). A percentile over all heights picked tooth rings and face pockets as the bore.
        floor_ = float(np.percentile(R_in, 5))
        if bore_steps:  # cfg.FLUSH_BORE_KEEP_STEPS (roller hubs lost their counterbores to the plain bore)
            R_med = revolve_profile(co, tri, d, centre, toothed=toothed)[2]
            R_in = np.maximum(_median_filter(R_med, 5), floor_)
        else:
            R_in = np.full_like(R_in, floor_)
    Rmax = float(R_out.max())
    tol = (tol_mm * MM) if tol_mm else (0.25 * MM if Rmax < 6 * MM else max(0.15 * MM, 0.03 * Rmax))
    outer = geo.douglas_peucker(stair_to_polyline(edges, R_out), tol)
    snap = (lambda p: _snap_profile(p, max(0.3 * MM, 0.5 * tol), max(0.2 * MM, 0.4 * tol))) if remodel else (lambda p: p)
    outer = snap(outer)
    if radial is None:
        radial = (edges[-1] - edges[0]) < 0.5 * Rmax
    rp = (_radial_profile(co, tri, d, centre, Rmax, float(R_in.min()) if keep_bore else 0.0, tol, remodel=remodel)
          if radial else None)
    if rp is not None:
        # only where it captures a real face recess (>15% less cross-section than the per-height envelope)
        env = [(edges[0], 0.0)] + list(map(tuple, outer)) + [(edges[-1], 0.0)]
        if abs(geo.poly_area(np.array(rp))) > 0.85 * abs(geo.poly_area(np.array(env))):
            rp = None
    if rp is not None:
        prof = rp
    elif keep_bore:
        inner = snap(geo.douglas_peucker(stair_to_polyline(edges, R_in), tol))
        # closed loop: outer forward, inner backward
        prof = list(map(tuple, outer)) + list(map(tuple, inner[::-1]))
    else:
        prof = [(edges[0], 0.0)] + list(map(tuple, outer)) + [(edges[-1], 0.0)]
    # drop consecutive duplicates
    clean = []
    for p in prof:
        if not clean or abs(p[0] - clean[-1][0]) > 1e-7 or abs(p[1] - clean[-1][1]) > 1e-7:
            clean.append(p)
    if len(clean) > 2 and abs(clean[0][0] - clean[-1][0]) < 1e-7 and abs(clean[0][1] - clean[-1][1]) < 1e-7:
        clean.pop()
    prof = clean
    if inset[0] or inset[1]:
        h0, h1 = min(p[0] for p in prof), max(p[0] for p in prof)
        prof = [((h + inset[0]) if abs(h - h0) < 1e-7 else (h - inset[1]) if abs(h - h1) < 1e-7 else h, r) for h, r in prof]
    if mat_rings and len(src_me.materials) > 1:
        prof = _material_breaks(src_me, d, centre, prof)
    n = nseg or geo.ngon_segments(Rmax / MM)
    u, w = geo.perp_basis(d)
    verts, faces = [], []
    ring = []  # per profile point: list of vertex indices (len 1 if on axis)
    eps = 1e-6
    for (h, r) in prof:
        base = centre + d * h
        if r < eps:
            ring.append([len(verts)])
            verts.append(base)
        else:
            idx = []
            for k in range(n):
                a = 2 * math.pi * k / n
                idx.append(len(verts))
                verts.append(base + (u * math.cos(a) + w * math.sin(a)) * r)
            ring.append(idx)
    smooth, sharp = set(), set()
    fseg = []
    m = len(prof)
    # turning angle at each profile vertex (for sharp ring edges)
    def turn(i):
        p0, p1, p2 = np.array(prof[i - 1]), np.array(prof[i]), np.array(prof[(i + 1) % m])
        a, b = p1 - p0, p2 - p1
        na, nb_ = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-9 or nb_ < 1e-9:
            return 180
        return math.degrees(math.acos(np.clip(a @ b / (na * nb_), -1, 1)))
    for i in range(m):
        j = (i + 1) % m
        A, B = ring[i], ring[j]
        if len(A) == 1 and len(B) == 1:
            continue
        # a segment is a "side" (smooth) if it is not perpendicular to the axis
        dh = abs(prof[j][0] - prof[i][0])
        dr = abs(prof[j][1] - prof[i][1])
        is_side = dh > 0.2 * dr
        for k in range(n):
            k2 = (k + 1) % n
            if len(A) == 1:
                f = (A[0], B[k], B[k2])
            elif len(B) == 1:
                f = (A[k], A[k2], B[0])
            else:
                f = (A[k], A[k2], B[k2], B[k])
            if is_side:
                smooth.add(len(faces))
            fseg.append(i)
            faces.append(f)
    for i in range(m):
        if len(ring[i]) > 1 and turn(i) > 30:
            R = ring[i]
            for k in range(n):
                sharp.add(frozenset((R[k], R[(k + 1) % n])))
    me = new_mesh_from(name, [tuple(v) for v in verts], faces, smooth, sharp)
    if len(fseg) == len(me.polygons):  # degenerate faces dropped by new_mesh_from -> no ring materials
        a = me.attributes.new("seg", 'INT', 'FACE')
        a.data.foreach_set("value", fseg)
    fix_normals(me)
    return me


def _radial_profile(co, tri, d, centre, Rmax, rbore, tol, bin_mm=0.25, remodel=False):
    """Profile of a disc-like part from its face heights at each radius (bottom face out, top face back in),
    so recessed webs, stepped hubs and face grooves survive. The per-height outer-radius profile fills them.
    remodel: face grooves narrower than REMODEL_WINDOW_MM filled, profile snapped to flat faces/cylinders."""
    pts, _ = geo.sample_surface(co, tri, 80000)
    pts = np.vstack([pts, co])
    u, w = geo.perp_basis(d)
    rel = pts - centre
    h, r = rel @ d, np.hypot(rel @ u, rel @ w)
    nb = int(np.clip((Rmax - rbore) / (bin_mm * MM), 8, 600))
    k = np.clip(((r - rbore) / max(Rmax - rbore, 1e-9) * nb).astype(int), 0, nb - 1)
    lo, hi = np.full(nb, np.nan), np.full(nb, np.nan)
    for i in np.unique(k):
        hs = h[k == i]
        if len(hs) >= 3:
            lo[i], hi[i] = np.percentile(hs, 1), np.percentile(hs, 99)
    ok = ~np.isnan(lo)
    if ok.sum() < 4:
        return None
    idx = np.arange(nb)
    lo[~ok] = np.interp(idx[~ok], idx[ok], lo[ok])
    hi[~ok] = np.interp(idx[~ok], idx[ok], hi[ok])
    lo, hi = _median_filter(lo, 5), _median_filter(hi, 5)
    if remodel:
        kw = min(int(round(REMODEL_WINDOW_MM / bin_mm)), nb // 3)
        hi = _closing(hi, kw)       # recesses in the top face are valleys of hi
        lo = -_closing(-lo, kw)     # ... and peaks of lo on the bottom face
    redges = rbore + (Rmax - rbore) * np.arange(nb + 1) / nb
    # (h, r) polylines: bottom face from the bore/axis outwards, top face from the rim back in
    bot = geo.douglas_peucker(np.array([(lo[i // 2], redges[i // 2 + i % 2]) for i in range(2 * nb)]), tol)
    top = geo.douglas_peucker(np.array([(hi[i // 2], redges[i // 2 + i % 2]) for i in range(2 * nb)]), tol)
    if remodel:  # here a face is a run of constant h along r: same snapping with the roles of h and r swapped
        sw = lambda p: np.array(p)[:, ::-1]
        bot = sw(_snap_profile(sw(bot), max(0.2 * MM, 0.4 * tol), max(0.3 * MM, 0.5 * tol)))
        top = sw(_snap_profile(sw(top), max(0.2 * MM, 0.4 * tol), max(0.3 * MM, 0.5 * tol)))
    prof = list(map(tuple, bot)) + list(map(tuple, top[::-1]))
    if rbore <= 0:
        prof = [(bot[0][0], 0.0)] + prof + [(top[0][0], 0.0)]
    return prof


def _material_breaks(src_me, d, centre, prof, bin_mm=0.25, min_band_mm=0.5):
    """Split the flat faces of a lathe profile where the CAD's colour changes (bearing races vs seal), so each
    lathe ring can take its own material. Without it a bearing face becomes one solid seal-coloured annulus."""
    co, tri, mat = geo.mesh_arrays(src_me)
    a, b, c = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
    cr = np.cross(b - a, c - a)
    nrm = cr / np.maximum(np.linalg.norm(cr, axis=1)[:, None], 1e-30)
    cap = np.abs(nrm @ d) > 0.95
    if cap.sum() < 3:
        return prof
    pts, _ = geo.sample_surface(co, tri[cap], 20000)
    # material of each sample: nearest cap triangle centroid (cheap and good enough at bin resolution)
    cen = (a[cap] + b[cap] + c[cap]) / 3
    from mathutils.kdtree import KDTree
    kd = KDTree(len(cen))
    for i, p in enumerate(cen):
        kd.insert(Vector(p), i)
    kd.balance()
    m_cap = mat[cap]
    sm = np.array([m_cap[kd.find(Vector(p))[1]] for p in pts])
    u, w = geo.perp_basis(d)
    rel = pts - centre
    h, r = rel @ d, np.hypot(rel @ u, rel @ w)
    P = [np.array(p) for p in prof]
    hmid = 0.5 * (min(p[0] for p in P) + max(p[0] for p in P))
    breaks = []
    for side in (h < hmid, h >= hmid):
        if side.sum() < 50:
            continue
        rs, ms, hs = r[side], sm[side], h[side]
        nb = int(np.ceil(rs.max() / (bin_mm * MM))) + 1
        bins = (rs / (bin_mm * MM)).astype(int)
        maj = np.full(nb, -1)
        for k in np.unique(bins):
            maj[k] = np.bincount(ms[bins == k]).argmax()
        runs = []  # (start bin, end bin, material) of runs at least min_band wide
        k = 0
        while k < nb:
            if maj[k] < 0:
                k += 1
                continue
            j = k
            while j + 1 < nb and maj[j + 1] in (maj[k], -1):
                j += 1
            runs.append((k, j, maj[k]))
            k = j + 1
        runs = [x for x in runs if (x[1] - x[0] + 1) * bin_mm >= min_band_mm]
        for (k0, k1, m0), (j0, j1, m1) in zip(runs, runs[1:]):
            if m0 != m1:
                rb = 0.5 * (k1 + 1 + j0) * bin_mm * MM
                near = np.abs(rs - rb) < 2 * bin_mm * MM
                breaks.append((float(np.median(hs[near])) if near.any() else float(np.median(hs)), rb))
    for hb, rb in breaks:
        # insert into the flat (radial) profile segment at this height that spans rb
        for i in range(len(P)):
            p, q = P[i], P[(i + 1) % len(P)]
            if abs(p[0] - q[0]) < 0.05 * MM and abs(p[0] - hb) < 1.0 * MM and min(p[1], q[1]) + 1e-5 < rb < max(p[1], q[1]) - 1e-5:
                P.insert(i + 1, np.array([p[0], rb]))
                break
    return [tuple(p) for p in P]


def transfer_materials_rings(src_me, dst_me, single=False):
    """Like transfer_materials, but every lathe ring (profile segment) gets one majority material,
    so faces over spokes vs. windows of the CAD part don't alternate colours.
    single (owner "for all bearings you can use one color"): the whole lathe takes ONE material, the area majority."""
    transfer_materials(src_me, dst_me)
    if single and len(src_me.materials) > 1 and len(dst_me.polygons):
        # area majority of the CAD part's own faces (slots are copied in order). Voting on the
        # lathe's faces let one big end-face annulus, centred on the shield, outvote the races: small flanged
        # bearings came out all shield-tan (CAD: 69-84% grey race).
        votes = {}
        for p in src_me.polygons:
            votes[p.material_index] = votes.get(p.material_index, 0.0) + p.area
        win = max(votes, key=votes.get)
        for p in dst_me.polygons:
            p.material_index = win
    if "seg" not in dst_me.attributes:
        return
    seg = [0] * len(dst_me.polygons)
    dst_me.attributes["seg"].data.foreach_get("value", seg)
    if len(src_me.materials) > 1:
        votes = {}
        for p, s in zip(dst_me.polygons, seg):
            votes.setdefault(s, {}).setdefault(p.material_index, 0.0)
            votes[s][p.material_index] += p.area
        win = {s: max(v, key=v.get) for s, v in votes.items()}
        for p, s in zip(dst_me.polygons, seg):
            p.material_index = win[s]
    dst_me.attributes.remove(dst_me.attributes["seg"])


# ---------------------------------------------------------------- 2D loops from cap faces

def cap_loops(me, d, level, tol=0.05 * MM):
    """Boundary loops (lists of 3D points) of the faces lying on plane h=level with normal parallel to d."""
    bm = bmesh.new()
    bm.from_mesh(me)
    dv = Vector(d)
    cap = set()
    for f in bm.faces:
        if abs(abs(f.normal.dot(dv)) - 1) < 1e-3 and all(abs(v.co.dot(dv) - level) < tol for v in f.verts):
            cap.add(f.index)
    area = sum(f.calc_area() for f in bm.faces if f.index in cap)
    bedges = [e for e in bm.edges if sum(1 for f in e.link_faces if f.index in cap) == 1]
    adj = {}
    for e in bedges:
        a, b = e.verts[0].index, e.verts[1].index
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, []).append(a)
    coords = {v.index: np.array(v.co[:]) for v in bm.verts}
    loops, used = [], set()
    for start in list(adj):
        if start in used:
            continue
        loop = [start]
        used.add(start)
        prev, cur = None, start
        while True:
            nxt = [x for x in adj[cur] if x != prev and x not in used]
            if not nxt:
                break
            prev, cur = cur, nxt[0]
            used.add(cur)
            loop.append(cur)
        if len(loop) >= 3:
            loops.append(np.array([coords[i] for i in loop]))
    bm.free()
    return loops, area


def to2d(pts, u, w):
    return np.c_[pts @ u, pts @ w]


def point_in_poly(pt, poly):
    x, y = pt
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-30) + xi:
            inside = not inside
        j = i
    return inside


def simplify_hole(loop2d, nseg_override=None):
    circ, c, r = _is_circle(loop2d)
    if circ:
        n = nseg_override or (6 if r < 4 * MM else 8 if r < 8 * MM else 12 if r < 15 * MM else 16)
        ang = np.arange(n) * 2 * math.pi / n + math.pi / n
        return np.c_[c[0] + r * np.cos(ang), c[1] + r * np.sin(ang)], ("circle", c, r)
    s = geo.douglas_peucker(loop2d, 0.4 * MM, closed=True)
    if len(s) < 3:
        s = loop2d[:: max(1, len(loop2d) // 4)]
    return s, ("poly", loop2d.mean(0), math.sqrt(abs(geo.poly_area(loop2d)) / math.pi))


def orient(loop2d, ccw):
    a = geo.poly_area(loop2d)
    if (a > 0) != ccw:
        return loop2d[::-1]
    return loop2d


def build_prism(name, outer2d, holes2d, u, w, d, h0, h1, smooth_angle=35):
    """Extrude a 2D region (outer loop + hole loops) between h0 and h1 along d."""
    outer2d = orient(np.asarray(outer2d), True)
    holes2d = [orient(np.asarray(h), False) for h in holes2d]
    loops = [outer2d] + holes2d
    verts, faces = [], []
    smooth, sharp = set(), set()
    base_top, base_bot = [], []
    for L in loops:
        bt, bb = [], []
        for p in L:
            bt.append(len(verts)); verts.append(u * p[0] + w * p[1] + d * h1)
            bb.append(len(verts)); verts.append(u * p[0] + w * p[1] + d * h0)
        base_top.append(bt); base_bot.append(bb)
    flat = [[(float(p[0]), float(p[1]), 0.0) for p in L] for L in loops]
    tris = tessellate_polygon(flat)
    offs = np.cumsum([0] + [len(L) for L in loops])
    def gidx(k, top):
        li = int(np.searchsorted(offs, k, side="right") - 1)
        return (base_top if top else base_bot)[li][k - offs[li]]
    for t in tris:
        faces.append(tuple(gidx(k, True) for k in t))
        faces.append(tuple(gidx(k, False) for k in t[::-1]))
    for li, L in enumerate(loops):
        n = len(L)
        T, B = base_top[li], base_bot[li]
        # turning angles for sharp side edges
        for i in range(n):
            j = (i + 1) % n
            smooth.add(len(faces))
            faces.append((T[i], B[i], B[j], T[j]))
            sharp.add(frozenset((T[i], T[j])))
            sharp.add(frozenset((B[i], B[j])))
            p0, p1, p2 = L[i - 1], L[i], L[j]
            a, b = p1 - p0, p2 - p1
            na, nb_ = np.linalg.norm(a), np.linalg.norm(b)
            ang = 180 if na < 1e-12 or nb_ < 1e-12 else math.degrees(math.acos(np.clip(a @ b / (na * nb_), -1, 1)))
            if ang > smooth_angle:
                sharp.add(frozenset((T[i], B[i])))
    me = new_mesh_from(name, [tuple(v) for v in verts], faces, smooth, sharp)
    fix_normals(me)
    return me


# ---------------------------------------------------------------- plates (2.5D)

def plate_axis(me):
    """Pick the thin axis whose two extreme planes carry the most cap area. Returns (d, h0, h1, frac) or None."""
    co, tri, _ = geo.mesh_arrays(me)
    a, b, c = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
    cr = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(cr, axis=1)
    tot = area.sum()
    nrm = cr / np.maximum(np.linalg.norm(cr, axis=1)[:, None], 1e-30)
    pts, _ = geo.sample_surface(co, tri, 4000)
    best = None
    for d in geo.candidate_axes(pts):
        h = co @ d
        h0, h1 = h.min(), h.max()
        t = h1 - h0
        ext = np.ptp(co - np.outer(h, d), axis=0).max()
        if t <= 0 or t > 0.2 * ext:
            continue
        th = np.c_[a @ d, b @ d, c @ d]
        par = np.abs(nrm @ d) > 0.999
        top = par & (np.abs(th - h1).max(1) < 5e-5)
        bot = par & (np.abs(th - h0).max(1) < 5e-5)
        at, ab = area[top].sum() / tot, area[bot].sum() / tot
        if at > 0.2 and ab > 0.2:
            score = at + ab
            if best is None or score > best[3]:
                best = (d, h0, h1, score)
    return best


def hole_samples(c, r, poly):
    pts = [np.asarray(c)]
    for k in range(6):
        a = k * math.pi / 3
        pts.append(np.asarray(c) + 0.6 * r * np.array([math.cos(a), math.sin(a)]))
    if poly is not None:
        pts = [p for p in pts if point_in_poly(p, poly)] or [np.asarray(c)]
    return pts


# object names of see-through parts (clear polycarbonate; the pipeline fills it from cfg.SEE_THROUGH_GROUPS):
# a clear panel lying on a tube or plate does not cover its holes, so these rays pass through it
SEE_THROUGH = set()


def ray_hits(scene, depsgraph, obj, p_local, d_local, dist):
    M = obj.matrix_world
    o = M @ Vector(p_local)
    dv = M.to_3x3() @ Vector(d_local * dist)
    L = dv.length
    if L <= 0:
        return False
    d = dv / L
    for _ in range(8):
        hit, loc, nrm, idx, hobj, _ = scene.ray_cast(depsgraph, o, d, distance=L)
        if not hit or hobj is None:
            return False
        if hobj.name not in SEE_THROUGH:
            return hobj.name != obj.name
        step = (loc - o).length + 1e-5
        L -= step
        if L <= 0:
            return False
        o = loc + d * 1e-5
    return False


def hole_state(scene, depsgraph, objs, u, w, d, h0, h1, c2d, r, poly):
    """'filled' if something occupies the hole volume (or both faces are covered) in every instance."""
    states = []
    for obj in objs:
        samples = hole_samples(c2d, r, poly)
        n = len(samples)
        through = cover_top = cover_bot = 0
        for s in samples:
            p = u * s[0] + w * s[1]
            if ray_hits(scene, depsgraph, obj, p + d * (h1 - 1e-5), -d, (h1 - h0) - 2e-5):
                through += 1
            if ray_hits(scene, depsgraph, obj, p + d * (h1 + 3e-3), -d, 3e-3 - 1e-5):
                cover_top += 1
            if ray_hits(scene, depsgraph, obj, p + d * (h0 - 3e-3), d, 3e-3 - 1e-5):
                cover_bot += 1
        need = max(1, int(math.ceil(n * 0.7)))
        filled = through >= need or (cover_top >= need and cover_bot >= need)
        states.append(filled)
    return all(states)


def analyse_plate(me):
    pa = plate_axis(me)
    if not pa:
        return None
    d, h0, h1, score = pa
    u, w = geo.perp_basis(d)
    top, _ = cap_loops(me, d, h1)
    bot, _ = cap_loops(me, d, h0)
    if not top or not bot:
        return None
    top2 = [to2d(L, u, w) for L in top]
    bot2 = [to2d(L, u, w) for L in bot]
    ta = [abs(geo.poly_area(L)) for L in top2]
    ba = [abs(geo.poly_area(L)) for L in bot2]
    it, ib = int(np.argmax(ta)), int(np.argmax(ba))
    outer = top2[it] if ta[it] >= ba[ib] else bot2[ib]
    th = [L for i, L in enumerate(top2) if i != it]
    bh = [L for i, L in enumerate(bot2) if i != ib]
    # loops inside the outer region that are themselves outer boundaries of islands are ignored (rare)
    holes = []
    for A in th:
        ca = A.mean(0)
        for B in bh:
            cb = B.mean(0)
            if point_in_poly(ca, B) or point_in_poly(cb, A):
                holes.append(A if abs(geo.poly_area(A)) <= abs(geo.poly_area(B)) else B)
                break
    return {"d": d, "h0": h0, "h1": h1, "u": u, "w": w, "outer": outer, "holes": holes, "score": score}


def build_plate(name, info, keep_holes, outline_tol_mm=None):
    outer = info["outer"]
    size = np.ptp(outer, axis=0).max()
    tol = (outline_tol_mm or max(0.3, size / MM * 0.0015)) * MM
    o2 = geo.douglas_peucker(outer, tol, closed=True)
    hl = []
    for H in keep_holes:
        s, _ = simplify_hole(H)
        if len(s) >= 3:
            hl.append(s)
    return build_prism(name, o2, hl, info["u"], info["w"], info["d"], info["h0"], info["h1"])


# ---------------------------------------------------------------- stepped 2.5D plates (pockets, multiple levels)
from mathutils.geometry import delaunay_2d_cdt


def column_axis(me):
    """Axis along which the part is a 'column solid' (faces parallel or perpendicular to it)."""
    co, tri, _ = geo.mesh_arrays(me)
    a, b, c = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
    cr = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(cr, axis=1)
    tot = area.sum()
    nrm = cr / np.maximum(np.linalg.norm(cr, axis=1)[:, None], 1e-30)
    pts, _ = geo.sample_surface(co, tri, 4000)
    best = None
    for d in geo.candidate_axes(pts):
        h = co @ d
        t = np.ptp(h)
        ext = np.ptp(co - np.outer(h, d), axis=0).max()
        if t <= 0:
            continue
        nd = np.abs(nrm @ d)
        par = nd > 0.999
        vert = nd < 0.02
        frac = area[par | vert].sum() / tot
        pfrac = area[par].sum() / tot
        th = np.c_[a @ d, b @ d, c @ d]
        prism = bool(np.all((np.abs(th[par] - h.min()) < 5e-5) | (np.abs(th[par] - h.max()) < 5e-5)))
        ok = frac > 0.96 and ((pfrac > 0.3 and t <= 0.25 * ext) or prism)
        if ok and (best is None or (prism, frac) > (best[2], best[1])):
            best = (d, frac, prism)
    return best


def level_loops(me, d, tol=5e-5, ignore_sign=False):
    """{(level, sign): [loop3d,...]} for faces parallel to d, grouped by height and facing.
    Loops follow face winding (directed boundary edges), so pinch points do not scramble them."""
    bm = bmesh.new()
    bm.from_mesh(me)
    dv = Vector(d)
    groups = {}
    for f in bm.faces:
        nd = f.normal.dot(dv)
        if abs(abs(nd) - 1) > 1e-3:
            continue
        h = sum(v.co.dot(dv) for v in f.verts) / len(f.verts)
        if any(abs(v.co.dot(dv) - h) > tol for v in f.verts):
            continue
        s = 1 if (nd > 0 or ignore_sign) else -1
        key = None
        for (lv, ss) in groups:
            if ss == s and abs(lv - h) < tol:
                key = (lv, ss)
                break
        if key is None:
            key = (h, s)
            groups[key] = []
        groups[key].append(f)
    out = {}
    coords = {v.index: np.array(v.co[:]) for v in bm.verts}
    for key, fs in groups.items():
        fset = set(f.index for f in fs)
        adj = {}
        for f in fs:
            for e in f.edges:
                if sum(1 for g in e.link_faces if g.index in fset) == 1:
                    a, b = e.verts[0].index, e.verts[1].index
                    adj.setdefault(a, []).append((b, e.index))
                    adj.setdefault(b, []).append((a, e.index))
        used = set()
        loops = []
        for start in list(adj):
            while any(ei not in used for _, ei in adj[start]):
                loop = [start]
                cur, prev = start, None
                while True:
                    cand = [(v, ei) for v, ei in adj[cur] if ei not in used]
                    if not cand:
                        break
                    # prefer not to bounce straight back; at pinch vertices take the first free edge
                    nxt = next(((v, ei) for v, ei in cand if v != prev), cand[0])
                    used.add(nxt[1])
                    prev, cur = cur, nxt[0]
                    if cur == start:
                        break
                    loop.append(cur)
                if len(loop) >= 3:
                    loops.append(np.array([coords[i] for i in loop]))
        out[key] = loops
    bm.free()
    return out


def parity_inside(points, loops):
    """Even-odd inside test of many points against a set of loops (vectorised)."""
    P = np.asarray(points)
    if len(P) == 0:
        return np.zeros(0, bool)
    E0 = np.vstack([L for L in loops])
    E1 = np.vstack([np.roll(L, -1, axis=0) for L in loops])
    inside = np.zeros(len(P), bool)
    CH = 4000
    for s in range(0, len(E0), CH):
        a, b = E0[s:s + CH], E1[s:s + CH]
        y = P[:, 1:2]
        x = P[:, 0:1]
        cond = (a[None, :, 1] > y) != (b[None, :, 1] > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xc = (b[None, :, 0] - a[None, :, 0]) * (y - a[None, :, 1]) / (b[None, :, 1] - a[None, :, 1]) + a[None, :, 0]
        cross = cond & (x < xc)
        inside ^= (cross.sum(1) % 2).astype(bool)
    return inside


def triangulate_region(loops, test_loops=None, inside_fn=None):
    """CDT of loops, keeping triangles inside by even-odd rule (against test_loops if given: the unsimplified
    outlines, when simplified loops may overlap each other). Returns (verts2d, tris)."""
    verts, edges = [], []
    for L in loops:
        b = len(verts)
        n = len(L)
        verts += [tuple(map(float, p)) for p in L]
        edges += [(b + i, b + (i + 1) % n) for i in range(n)]
    vout, eout, fout, *_ = delaunay_2d_cdt([Vector(v) for v in verts], edges, [], 0, 1e-7)
    V = np.array([v[:2] for v in vout])
    F = [f for f in fout if len(f) == 3]
    if not F:
        return V, []
    C = np.array([V[list(f)].mean(0) for f in F])
    keep = inside_fn(C) if inside_fn is not None else parity_inside(C, test_loops if test_loops is not None else loops)
    return V, [F[i] for i in range(len(F)) if keep[i]]


def analyse_stepped(me):
    ca = column_axis(me)
    if not ca:
        return None
    d = ca[0]
    u, w = geo.perp_basis(d)
    raw = level_loops(me, d)
    levels = {}
    for (h, s), loops in raw.items():
        levels[(h, s)] = [to2d(L, u, w) for L in loops if abs(geo.poly_area(to2d(L, u, w))) > 1e-10]
    ups = sorted([k for k in levels if k[1] > 0], key=lambda k: -k[0])
    dns = sorted([k for k in levels if k[1] < 0], key=lambda k: k[0])
    if not ups or not dns:
        return None
    return {"d": d, "u": u, "w": w, "levels": levels, "ups": ups, "dns": dns, "frac": ca[1]}


def _is_circle(L):
    c, r, dev = geo.fit_circle(L)
    area = abs(geo.poly_area(L))
    return (dev < max(0.03 * MM, 0.01 * r) and abs(area - math.pi * r * r) < 0.15 * math.pi * r * r), c, r


def _circle_ngon(c, r, circle_sides=None):
    """n-gon for a circle: hole tiers by radius, or circle_sides(r_mm)."""
    n = circle_sides(r / MM) if circle_sides else (6 if r < 4 * MM else 8 if r < 8 * MM else 12 if r < 15 * MM else 16 if r < 30 * MM else 24)
    c, r = np.round(c, 6), round(float(r), 6)
    ang = np.arange(n) * 2 * math.pi / n + math.pi / n
    return np.c_[c[0] + r * np.cos(ang), c[1] + r * np.sin(ang)]


def _simplify_loop(L, tol, circle_sides=None):
    """Circles become n-gons (hole tiers, or circle_sides(r_mm) for any radius); other loops are Douglas-Peucker'd."""
    circ, c, r = _is_circle(L)
    if circ and (circle_sides or r < 40 * MM):
        n = circle_sides(r / MM) if circle_sides else (6 if r < 4 * MM else 8 if r < 8 * MM else 12 if r < 15 * MM else 16 if r < 30 * MM else 24)
        c, r = np.round(c, 6), round(float(r), 6)  # the same hole on two levels must give the same n-gon
        ang = np.arange(n) * 2 * math.pi / n + math.pi / n
        return np.c_[c[0] + r * np.cos(ang), c[1] + r * np.sin(ang)]
    s = geo.douglas_peucker(_canonical_loop(L), tol, closed=True)
    return s if len(s) >= 3 else None


def _loop_max_dist(P, Q):
    """Largest distance from the points P to the closed polyline Q (2D, chunked)."""
    A = Q; AB = np.roll(Q, -1, axis=0) - A; L2 = (AB ** 2).sum(1) + 1e-30; m = 0.0
    for i in range(0, len(P), 256):
        p = P[i:i + 256, None, :]
        t = np.clip(((p - A) * AB).sum(2) / L2, 0, 1)
        m = max(m, float(np.sqrt(((p - (A + t[..., None] * AB)) ** 2).sum(2)).min(1).max()))
    return m


def _same_outline(Lc, seen, tol=0.05 * MM):
    """One outline on two levels, tessellated differently (a gear rim: top 4,742 points, bottom 6,521, within
    0.02 mm): simplified separately they shared no vertices, so the walls (top outline) and the bottom cap (bottom
    outline) left a sliver of open loops, and fill_planar_boundaries' n-gons over that non-convex sliver spanned the
    tooth gaps (~800 flap triangles, both orientations). A loop matching an earlier one (area within 0.2%, bbox and
    two-way max distance < tol) reuses it, so both levels get the same simplified vertices."""
    a = abs(geo.poly_area(Lc)); lo, hi = Lc.min(0), Lc.max(0)
    for L0, a0, lo0, hi0 in seen:
        if (abs(a - a0) <= 0.002 * a0 and np.abs(lo - lo0).max() < tol and np.abs(hi - hi0).max() < tol
                and _loop_max_dist(Lc, L0) < tol and _loop_max_dist(L0, Lc) < tol):
            return L0
    seen.append((Lc, a, lo, hi))
    return Lc


def _canonical_loop(L):
    """Same outline -> same simplification: CCW, starting at the lowest-leftmost point. Douglas-Peucker on a
    closed loop depends on where the loop starts, so the top and bottom caps of a plate (and the walls built from
    them) came out with different vertices and left open seams."""
    L = np.asarray(L, float)
    if geo.poly_area(L) < 0:
        L = L[::-1]
    k = int(np.lexsort((np.round(L[:, 1], 6), np.round(L[:, 0], 6)))[0])
    return np.roll(L, -k, axis=0)


def plan_holes(info, min_diam_mm=3.5):
    """Find through-holes: up-loops whose outer side has no material at any level. Returns list of
    (up_key, loop_index, centre2d, eq_radius, loop2d, matching (dn_key, idx) list)."""
    levels, ups, dns = info["levels"], info["ups"], info["dns"]
    holes = []
    for k in ups:
        for i, L in enumerate(levels[k]):
            area = geo.poly_area(L)
            c = L.mean(0)
            # material side test: a point slightly inside the loop polygon
            inside_pt = c
            if not point_in_poly(c, L):
                continue
            # is the loop interior empty in every level? -> through hole
            empty = True
            for kk in ups + dns:
                lps = levels[kk]
                if parity_inside([inside_pt], lps)[0]:
                    empty = False
                    break
            if not empty:
                continue
            req = math.sqrt(abs(area) / math.pi)
            match = []
            for kk in dns:
                for j, M in enumerate(levels[kk]):
                    am = abs(geo.poly_area(M))
                    if not (point_in_poly(c, M) and 0.3 * abs(area) < am < 3.0 * abs(area)):
                        continue
                    cm = M.mean(0)
                    if not point_in_poly(cm, M):
                        continue
                    if any(parity_inside([cm], levels[k2])[0] for k2 in ups + dns if levels[k2]):
                        continue  # M encloses material -> it is an outline, not a hole
                    match.append((kk, j))
            holes.append((k, i, c, req, L, match))
    return holes


def build_stepped(name, info, drop, tol_mm=0.45, circle_sides=None):
    """drop: set of (key, idx) loops to remove (filled/small holes). tol_mm: outline simplification;
    circle_sides(r_mm): side count for circular loops of any size (default: hole tiers, circles < 40 mm only)."""
    d, u, w = info["d"], info["u"], info["w"]
    levels, ups, dns = info["levels"], info["ups"], info["dns"]
    allpts = np.vstack([L for k in levels for L in levels[k]])
    size = np.ptp(allpts, axis=0).max()
    tol = tol_mm * MM
    # Simplify every level consistently: an outline vertex kept on one level is kept on every level that has it,
    # so caps and the walls between them share vertices (separately simplified levels left open seams).
    key = lambda p: (round(float(p[0]), 6), round(float(p[1]), 6))
    first, kept = {}, set()
    seen_loops = []  # outlines already simplified on another level (_same_outline)
    fits = []  # one (centre, radius) per physical hole: the per-level fits differ by a few um, so their n-gons did not
    #            match, the walls left open seams and fill_planar_boundaries patched part of the hole
    for k in levels:
        for i, L in enumerate(levels[k]):
            if (k, i) in drop:
                continue
            circ, c, r = _is_circle(L)
            if circ and (circle_sides or r < 40 * MM):
                same = [f for f in fits if np.linalg.norm(f[0] - c) < 0.05 * MM and abs(f[1] - r) < 0.05 * MM]
                if same:
                    c, r = same[0]
                else:
                    fits.append((np.asarray(c, float), float(r)))
                first[(k, i)] = ("circle", _circle_ngon(c, r, circle_sides))
                continue
            Lc = _same_outline(_canonical_loop(L), seen_loops)
            sp = geo.douglas_peucker(Lc, tol, closed=True)
            kept.update(key(p) for p in sp)
            first[(k, i)] = ("poly", Lc)
    S = {k: [] for k in levels}
    for (k, i), (kind, L) in first.items():
        if kind == "poly":
            L = L[[key(p) in kept for p in L]]
        if L is not None and len(L) >= 3:
            S[k].append(L)
    verts, faces = [], []

    def add_v(p2, h):
        verts.append(u * p2[0] + w * p2[1] + d * h)
        return len(verts) - 1

    # caps
    for k in levels:
        h, s = k
        if not S[k]:
            continue
        V, F = triangulate_region(S[k])
        ids = [add_v(p, h) for p in V]
        for f in F:
            a2, b2, c2 = V[f[0]], V[f[1]], V[f[2]]
            ccw = (b2[0] - a2[0]) * (c2[1] - a2[1]) - (b2[1] - a2[1]) * (c2[0] - a2[0]) > 0
            f = [ids[x] for x in f]
            if ccw != (s > 0):
                f = f[::-1]
            faces.append(tuple(f))

    def top_at(p):
        for k in ups:
            if S[k] and parity_inside([p], S[k])[0]:
                return k[0]
        return None

    def bot_at(p):
        for k in dns:
            if S[k] and parity_inside([p], S[k])[0]:
                return k[0]
        return None

    off = 0.4 * MM
    def _orient_wall(q, out2d):
        P = [np.asarray(verts[i]) for i in q]
        nrm3 = np.cross(P[1] - P[0], P[3] - P[0])
        out3 = u * out2d[0] + w * out2d[1]
        return tuple(q) if nrm3 @ out3 > 0 else tuple(q[::-1])

    # walls from up-loops down
    for k in ups:
        h = k[0]
        for L in S[k]:
            n = len(L)
            mids = (L + np.roll(L, -1, axis=0)) / 2
            tang = np.roll(L, -1, axis=0) - L
            nrm = np.c_[tang[:, 1], -tang[:, 0]]
            nrm /= np.maximum(np.linalg.norm(nrm, axis=1)[:, None], 1e-30)
            pa = mids + nrm * off
            pb = mids - nrm * off
            ina = parity_inside(pa, S[k])
            for e in range(n):
                outside = pb[e] if ina[e] else pa[e]
                inside = pa[e] if ina[e] else pb[e]
                t = top_at(outside)
                if t is not None and t < h - 1e-6:
                    bot = t
                elif t is None:
                    bb = bot_at(inside)
                    bot = bb if bb is not None else dns[0][0]
                else:
                    continue
                if bot >= h - 1e-6:
                    continue
                p0, p1 = L[e], L[(e + 1) % n]
                a, b = add_v(p0, h), add_v(p1, h)
                c_, d_ = add_v(p1, bot), add_v(p0, bot)
                faces.append(_orient_wall((a, b, c_, d_), outside - inside))
    # walls on the underside where the bottom steps up
    for k in dns:
        h = k[0]
        for L in S[k]:
            n = len(L)
            mids = (L + np.roll(L, -1, axis=0)) / 2
            tang = np.roll(L, -1, axis=0) - L
            nrm = np.c_[tang[:, 1], -tang[:, 0]]
            nrm /= np.maximum(np.linalg.norm(nrm, axis=1)[:, None], 1e-30)
            pa = mids + nrm * off
            pb = mids - nrm * off
            ina = parity_inside(pa, S[k])
            for e in range(n):
                outside = pb[e] if ina[e] else pa[e]
                inside = pa[e] if ina[e] else pb[e]
                b2 = bot_at(outside)
                if b2 is None or b2 <= h + 1e-6:
                    continue
                p0, p1 = L[e], L[(e + 1) % n]
                a, b = add_v(p0, h), add_v(p1, h)
                c_, d_ = add_v(p1, b2), add_v(p0, b2)
                faces.append(_orient_wall((a, b, c_, d_), outside - inside))
    me = new_mesh_from(name, [tuple(v) for v in verts], faces)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
    bm.to_mesh(me)
    bm.free()
    fix_t_junctions(me)
    if open_edges(me):
        fill_planar_boundaries(me)
    me.update()
    return me


def fill_planar_boundaries(me, tol=5e-5):
    """Close boundary loops that lie in one plane with a face wound against its neighbours. In a stepped plate a
    missing wall or cap patch leaves exactly such a loop (25 plates on one robot were rejected for 4-149 open edges and
    went to Collapse, which chewed their holes into triangles). Non-planar loops are left alone; the caller's fit
    check against the CAD rejects a fill that closed a real opening. Returns loops filled."""
    bm = bmesh.new()
    bm.from_mesh(me)
    bnd = [e for e in bm.edges if len(e.link_faces) == 1]  # index order: a set iterates by address (run-to-run)
    adj = collections.defaultdict(list)
    for e in bnd:
        for v in e.verts:
            adj[v].append(e)
    used, filled = set(), 0
    for e0 in bnd:
        if e0 in used:
            continue
        # walk the boundary loop
        loop, e, v = [], e0, e0.verts[0]
        ok = True
        while True:
            used.add(e)
            loop.append(v)
            v = e.other_vert(v)
            nxt = [x for x in adj[v] if x not in used]
            if v is loop[0]:
                break
            if len(adj[v]) != 2 or not nxt:
                ok = False
                break
            e = nxt[0]
        if not ok or len(loop) < 3:
            continue
        P = np.array([x.co[:] for x in loop])
        c = P.mean(0)
        if np.linalg.svd(P - c, compute_uv=False)[-1] / np.sqrt(len(P)) > tol:
            continue
        try:
            f = bm.faces.new(loop)
        except ValueError:
            continue
        # wind against the neighbour across the first edge (consistent winding = opposite traversal)
        a, b2 = loop[0], loop[1]
        ed = bm.edges.get((a, b2))
        other = [g for g in ed.link_faces if g is not f] if ed else []
        if other:
            vs = list(other[0].verts)
            i = vs.index(a)
            if vs[(i + 1) % len(vs)] is b2:  # neighbour runs a->b too: flip ours
                f.normal_flip()
        filled += 1
    if filled:
        bmesh.ops.triangulate(bm, faces=[f for f in bm.faces if len(f.verts) > 4], quad_method='BEAUTY', ngon_method='BEAUTY')
        bm.to_mesh(me)
        me.update()
    bm.free()
    return filled


def fix_t_junctions(me, eps=2e-6, rounds=3, fill=True):
    """Close seams where a boundary vertex sits on another face's boundary edge (a wall meeting a cap edge the
    triangulation split): split that edge at the vertex and weld. Unity culls backfaces, so such seams show as
    see-through slits. Returns the number of open edges left."""
    from mathutils.kdtree import KDTree
    bm = bmesh.new()
    bm.from_mesh(me)
    for _ in range(rounds):
        bnd = [e for e in bm.edges if len(e.link_faces) == 1]
        if not bnd:
            break
        bv = list({v for e in bnd for v in e.verts})
        kd = KDTree(len(bv))
        for i, v in enumerate(bv):
            kd.insert(v.co, i)
        kd.balance()
        todo = []
        for e in bnd:
            a, b = e.verts
            ab = b.co - a.co
            L2 = ab.length_squared
            if L2 <= 0:
                continue
            hits = []
            for co, i, _ in kd.find_range((a.co + b.co) / 2, math.sqrt(L2) / 2 + eps):
                v = bv[i]
                if v is a or v is b:
                    continue
                t = (v.co - a.co).dot(ab) / L2
                if 1e-6 < t < 1 - 1e-6 and (a.co + ab * t - v.co).length < eps:
                    hits.append((t, v))
            if hits:
                todo.append((e, a, sorted(hits, key=lambda x: x[0])))
        if not todo:
            break
        for e, a, hits in todo:
            if not e.is_valid:
                continue
            b = e.other_vert(a)
            cur, start, t0 = e, a, 0.0
            for t, v in hits:
                _, nv = bmesh.utils.edge_split(cur, start, (t - t0) / (1 - t0))
                nv.co = v.co.copy()
                cur = next((x for x in nv.link_edges if x.other_vert(nv) is b), None)
                if cur is None:
                    break
                start, t0 = nv, t
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=eps)
    if fill:
        # close leftover *slits* (thin boundary loops); anything wider is a real gap and is left for the caller
        # to reject (filling a window loop spans the window with slivers and breaks the winding)
        bnd = [e for e in bm.edges if len(e.link_faces) == 1]
        seen, slits = set(), []
        for e0 in bnd:
            if e0 in seen:
                continue
            comp, stack = [], [e0]
            seen.add(e0)
            while stack:
                e = stack.pop()
                comp.append(e)
                for v in e.verts:
                    for e2 in v.link_edges:
                        if e2 not in seen and len(e2.link_faces) == 1:
                            seen.add(e2)
                            stack.append(e2)
            P = np.array([v.co[:] for e in comp for v in e.verts])
            if len(P) >= 3:
                ext = np.sort(np.linalg.svd(P - P.mean(0), compute_uv=False))[::-1] / math.sqrt(len(P))
                if ext[1] < 0.5 * MM:
                    slits.extend(comp)
        if slits:
            bmesh.ops.holes_fill(bm, edges=slits, sides=0)
            bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    left = sum(1 for e in bm.edges if len(e.link_faces) == 1)
    bm.to_mesh(me)
    bm.free()
    return left


# ---------------------------------------------------------------- procedural box tubes with hex holes

# cfg.TUBE_MITRED, set by pipeline.py: a tube without any square end face (mitred ends) takes its walls from the
# inner side faces; its wall outlines carry the cut, so build_tube builds it as slabs (_build_tube_slabs) and keeps the
# mitre. Off: such tubes are rejected and decimated (Collapse plugs their small holes).
TUBE_MITRED = False


def analyse_tube(me):
    co, tri, _ = geo.mesh_arrays(me)
    mn, mx = co.min(0), co.max(0)
    dims = mx - mn
    L = int(np.argmax(dims))
    others = [a for a in range(3) if a != L]
    c = (mn + mx) / 2
    E = np.eye(3)
    # wall thickness from the end-cap faces
    lv = level_loops(me, E[L])
    endk = [k for k in lv if abs(k[0] - mx[L]) < 1e-4 and k[1] > 0]
    t = None
    if endk and len(lv[endk[0]]) >= 2:
        pts = [L_[:, others] for L_ in lv[endk[0]]]
        areas = [abs(geo.poly_area(p)) for p in pts]
        inner = pts[int(np.argsort(areas)[-2])]
        ih = np.abs(inner - c[others]).max(0)
        t = (dims[others] / 2) - ih
    if endk and (t is None or np.any(t <= 0.3e-3) or np.any(t > 0.3 * dims[others])):
        # end cap present but incomplete in the CAD tessellation (chassis rails: a C-shaped cap, one wall's end
        # face missing): take the walls from the inner side faces instead. No square end face at all means a mitred
        # end (arm tubes): those stay rejected, a straight box would lose the cut.
        t = _tube_walls_from_sides(co, tri, mn, mx, dims, others)
    mitred = False
    if not endk and TUBE_MITRED:
        t = _tube_walls_from_sides(co, tri, mn, mx, dims, others)
        mitred = True
    if t is None or np.any(t <= 0.3e-3) or np.any(t > 0.3 * dims[others]):
        return None
    walls = []
    for j, k in enumerate(others):
        m = others[1 - j]
        grp = level_loops(me, E[k])
        for s in (1, -1):
            outer_lvl = mx[k] if s > 0 else mn[k]
            key = [kk for kk in grp if abs(kk[0] - outer_lvl) < 1e-4 and kk[1] == s]
            holes, outline = [], None
            if key:
                lps = grp[key[0]]
                P = [lp[:, [L, m]] for lp in lps]
                ar = [abs(geo.poly_area(p)) for p in P]
                big = int(np.argmax(ar))
                holes = [p for i, p in enumerate(P) if i != big]
                outline = P[big]
            walls.append({"k": k, "m": m, "s": s, "holes": holes, "outline": outline})
    info = {"L": L, "others": others, "c": c, "half": dims / 2, "t": dict(zip(others, t)), "walls": walls}
    if mitred and (any(w["outline"] is None for w in walls) or all(_wall_outline(info, w) is None for w in walls)):
        return None  # no end face and no cut in the wall outlines: not a plain mitred tube
    return info


_POCKET_BVH = {}


def tube_hole_is_pocket(me, info, wall, poly2d):
    """(cfg.TUBE_THROUGH_TEST) A wall "hole" that is only a shallow pocket outline engraved in the wall (truss
    pattern on a climber support): rays shot into it from outside hit the pocket floor within the wall thickness,
    while through a real hole they cross the tube to the opposite wall. `me` is the tube mesh in its analysis frame."""
    key = me.as_pointer()
    if key not in _POCKET_BVH:
        co, tri, _ = geo.mesh_arrays(me)
        _POCKET_BVH.clear()
        _POCKET_BVH[key] = BVHTree.FromPolygons([Vector(c) for c in co], [tuple(t) for t in tri])
    bvh = _POCKET_BVH[key]
    L, k, m, s = info["L"], wall["k"], wall["m"], wall["s"]
    c, half, t = info["c"], info["half"], info["t"][k]
    P = np.asarray(poly2d, float)[:, :2]
    lo, hi = P.min(0), P.max(0)
    g = np.stack(np.meshgrid(np.linspace(lo[0], hi[0], 7)[1:-1], np.linspace(lo[1], hi[1], 7)[1:-1]), -1).reshape(-1, 2)
    pts = [q for q in g if point_in_poly(q, P)][:12]
    if not pts:
        return False
    floor = 0
    for q in pts:
        o = np.zeros(3)
        o[L], o[m] = q[0], q[1]
        o[k] = c[k] + s * (half[k] + 1e-3)
        d = np.zeros(3)
        d[k] = -s
        hit = bvh.ray_cast(Vector(o), Vector(d), 1e-3 + t + 0.3e-3)
        floor += hit[0] is not None
    return floor > len(pts) / 2


def tube_hole_covered(scene, depsgraph, obj, info, wall, poly2d):
    L, k, m, s = info["L"], wall["k"], wall["m"], wall["s"]
    c, half = info["c"], info["half"]
    cc, r, dev = geo.fit_circle(poly2d)
    if dev > max(0.03 * MM, 0.01 * r):
        cc = poly2d.mean(0)
        r = math.sqrt(abs(geo.poly_area(poly2d)) / math.pi)
    hits = 0
    samples = hole_samples(cc, r, None)
    for p2 in samples:
        p = np.zeros(3)
        p[L], p[m] = p2[0], p2[1]
        p[k] = c[k] + s * (half[k] + 3e-3)
        dvec = np.zeros(3)
        dvec[k] = -s
        if ray_hits(scene, depsgraph, obj, p, dvec, 3e-3 + 1e-4):
            hits += 1
    return hits >= len(samples) - 1


def _clip_band(poly, lo, hi):
    """Polygon (N, 2) clipped to lo <= y <= hi (Sutherland-Hodgman)."""
    def clip(P, keep, y0):
        out = []
        for i in range(len(P)):
            a, b = P[i - 1], P[i]
            if keep(b):
                if not keep(a):
                    out.append(a + (b - a) * ((y0 - a[1]) / (b[1] - a[1])))
                out.append(b)
            elif keep(a):
                out.append(a + (b - a) * ((y0 - a[1]) / (b[1] - a[1])))
        return np.array(out)
    P = clip(np.asarray(poly, float), lambda p: p[1] >= lo - 1e-9, lo)
    if len(P) >= 3:
        P = clip(P, lambda p: p[1] <= hi + 1e-9, hi)
    return P if len(P) >= 3 else None


def _wall_outline(info, wall):
    """The wall's own outline (L, m) when it is not the full rectangle (end notches, mitred ends), else None."""
    L, m = info["L"], wall["m"]
    c, half = info["c"], info["half"]
    O = wall.get("outline")
    if O is None:
        return None
    s = _simplify_loop(O, 0.4 * MM)
    if s is None:
        return None
    full = (len(s) == 4 and abs(s[:, 0].min() - (c[L] - half[L])) < 3e-4 and abs(s[:, 0].max() - (c[L] + half[L])) < 3e-4
            and abs(s[:, 1].min() - (c[m] - half[m])) < 3e-4 and abs(s[:, 1].max() - (c[m] + half[m])) < 3e-4)
    return None if full else s


def build_tube(name, info, keep):
    """keep: {wall_index: [hole polys]}. Hex holes through each wall, hollow rect tube."""
    L, (A, B) = info["L"], info["others"]
    c, half, t = info["c"], info["half"], info["t"]
    if any(_wall_outline(info, w) is not None for w in info["walls"]):
        return _build_tube_slabs(name, info, keep)
    verts, faces = [], []

    def P(coords):
        p = np.array(c, float)
        for ax, v in coords.items():
            p[ax] = v
        verts.append(p)
        return len(verts) - 1

    def add_face(idx, want):
        Q = [verts[i] for i in idx]
        n = np.cross(Q[1] - Q[0], Q[2] - Q[0])
        faces.append(tuple(idx) if n @ want > 0 else tuple(idx[::-1]))

    E = np.eye(3)
    for wi, wall in enumerate(info["walls"]):
        k, m, s = wall["k"], wall["m"], wall["s"]
        holes = []
        for H in keep.get(wi, []):
            sh = _simplify_loop(H, 0.4 * MM)
            if sh is not None and len(sh) >= 3:
                holes.append(sh)
        for inner in (False, True):
            lvl = c[k] + s * (half[k] - (t[k] if inner else 0))
            hm = half[m] - (t[m] if inner else 0)
            rect = np.array([[c[L] - half[L], c[m] - hm], [c[L] + half[L], c[m] - hm],
                             [c[L] + half[L], c[m] + hm], [c[L] - half[L], c[m] + hm]])
            V, F = triangulate_region([rect] + holes)
            ids = [P({L: v[0], m: v[1], k: lvl}) for v in V]
            want = E[k] * (s if not inner else -s)
            for f in F:
                add_face([ids[x] for x in f], want)
        o_l, i_l = c[k] + s * half[k], c[k] + s * (half[k] - t[k])
        for H in holes:
            ctr = H.mean(0)
            n = len(H)
            for e in range(n):
                p0, p1 = H[e], H[(e + 1) % n]
                q = [P({L: p0[0], m: p0[1], k: o_l}), P({L: p1[0], m: p1[1], k: o_l}),
                     P({L: p1[0], m: p1[1], k: i_l}), P({L: p0[0], m: p0[1], k: i_l})]
                mid = (p0 + p1) / 2
                want = np.zeros(3)
                want[L], want[m] = ctr[0] - mid[0], ctr[1] - mid[1]
                add_face(q, want)
    # end caps (rect annulus)
    for se in (1, -1):
        lvl = c[L] + se * half[L]
        o = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        outer = [P({L: lvl, A: c[A] + a * half[A], B: c[B] + b * half[B]}) for a, b in o]
        inner = [P({L: lvl, A: c[A] + a * (half[A] - t[A]), B: c[B] + b * (half[B] - t[B])}) for a, b in o]
        for i in range(4):
            j = (i + 1) % 4
            add_face([outer[i], outer[j], inner[j], inner[i]], E[L] * se)
    me = new_mesh_from(name, [tuple(v) for v in verts], faces)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
    bm.to_mesh(me)
    bm.free()
    me.update()
    return me



def _build_tube_slabs(name, info, keep):
    """Tube whose walls are not all full rectangles (an arm frame: one wall cut back 51 mm at an end, the
    side walls mitred to it). Each wall is a solid slab extruded from its own outline; the walls normal to the
    first cross axis own the corners, the other pair is clipped between them, so nothing overlaps."""
    L, (A, B) = info["L"], info["others"]
    c, half, t = info["c"], info["half"], info["t"]
    verts, faces = [], []

    def P(coords):
        p = np.array(c, float)
        for ax, v in coords.items():
            p[ax] = v
        verts.append(p)
        return len(verts) - 1

    def add_face(idx, want):
        Q = [verts[i] for i in idx]
        n = np.cross(Q[1] - Q[0], Q[2] - Q[0])
        faces.append(tuple(idx) if n @ want > 0 else tuple(idx[::-1]))

    E = np.eye(3)
    for wi, wall in enumerate(info["walls"]):
        k, m, s = wall["k"], wall["m"], wall["s"]
        O = _wall_outline(info, wall)
        if O is None:
            O = np.array([[c[L] - half[L], c[m] - half[m]], [c[L] + half[L], c[m] - half[m]],
                          [c[L] + half[L], c[m] + half[m]], [c[L] - half[L], c[m] + half[m]]])
        if k == B:  # clipped between the A walls
            O = _clip_band(O, c[m] - (half[m] - t[m]), c[m] + (half[m] - t[m]))
            if O is None:
                continue
        if geo.poly_area(O) < 0:
            O = O[::-1]
        holes = []
        for H in keep.get(wi, []):
            sh = _simplify_loop(H, 0.4 * MM)
            if sh is not None and len(sh) >= 3:
                holes.append(sh)
        lv_o, lv_i = c[k] + s * half[k], c[k] + s * (half[k] - t[k])
        V, F = triangulate_region([O] + holes)
        for lvl, want in ((lv_o, E[k] * s), (lv_i, -E[k] * s)):
            ids = [P({L: v[0], m: v[1], k: lvl}) for v in V]
            for f in F:
                add_face([ids[x] for x in f], want)
        for e in range(len(O)):  # slab rim, outward from the outline (CCW)
            p0, p1 = O[e], O[(e + 1) % len(O)]
            q = [P({L: p0[0], m: p0[1], k: lv_o}), P({L: p1[0], m: p1[1], k: lv_o}),
                 P({L: p1[0], m: p1[1], k: lv_i}), P({L: p0[0], m: p0[1], k: lv_i})]
            want = np.zeros(3)
            want[L], want[m] = p1[1] - p0[1], p0[0] - p1[0]
            add_face(q, want)
        for H in holes:
            ctr = H.mean(0)
            for e in range(len(H)):
                p0, p1 = H[e], H[(e + 1) % len(H)]
                q = [P({L: p0[0], m: p0[1], k: lv_o}), P({L: p1[0], m: p1[1], k: lv_o}),
                     P({L: p1[0], m: p1[1], k: lv_i}), P({L: p0[0], m: p0[1], k: lv_i})]
                mid = (p0 + p1) / 2
                want = np.zeros(3)
                want[L], want[m] = ctr[0] - mid[0], ctr[1] - mid[1]
                add_face(q, want)
    me = new_mesh_from(name, [tuple(v) for v in verts], faces)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
    bm.to_mesh(me)
    bm.free()
    me.update()
    return me


# ---------------------------------------------------------------- belts, boxes

def _dist_to_polyline(P, poly):
    A = poly
    B = np.roll(poly, -1, axis=0)
    AB = B - A
    L2 = np.maximum((AB ** 2).sum(1), 1e-18)
    t = np.clip(((P[:, None, :] - A[None]) * AB[None]).sum(-1) / L2[None], 0, 1)
    Q = A[None] + t[..., None] * AB[None]
    return np.sqrt(((P[:, None, :] - Q) ** 2).sum(-1)).min(1)


def _dedupe_loop(P, eps=1e-6):
    out = [P[0]]
    for p in P[1:]:
        if np.linalg.norm(p - out[-1]) > eps:
            out.append(p)
    if len(out) > 2 and np.linalg.norm(out[0] - out[-1]) <= eps:
        out.pop()
    return np.array(out)


def build_belt(name, me):
    co, tri, _ = geo.mesh_arrays(me)
    pts, _ = geo.sample_surface(co, tri, 4000)
    _, ax = geo.pca_axes(pts)
    d = geo.snap_axis(ax[2])
    u, w = geo.perp_basis(d)
    h = co @ d
    h0, h1 = h.min(), h.max()
    lv = level_loops(me, d, tol=1e-4, ignore_sign=True)
    top = [k for k in lv if abs(k[0] - h1) < 2e-4] + [k for k in lv if abs(k[0] - h0) < 2e-4]
    if not top:
        return None
    # lettering on one face adds loops there (80T belt: 5 on top): use a face with just outer + inner
    top.sort(key=lambda k: len(lv[k]) != 2)
    loops = [to2d(L, u, w) for L in lv[top[0]]]
    loops = [L for L in loops if abs(geo.poly_area(L)) > 1e-6]
    if len(loops) < 2:
        return None
    order = np.argsort([-abs(geo.poly_area(L)) for L in loops])
    outer = orient(loops[order[0]], True)
    inner = loops[order[1]]
    if not point_in_poly(inner.mean(0), outer):
        return None
    # belt body = back + land: take the near (land) side of the inner loop, not the tooth tips
    th = float(np.percentile(_dist_to_polyline(inner, outer), 35))
    o2 = _dedupe_loop(orient(geo.douglas_peucker(outer, 0.3 * MM, closed=True), True))
    n = len(o2)
    inn = []
    for i in range(n):
        p0, p1, p2 = o2[i - 1], o2[i], o2[(i + 1) % n]
        e0, e1 = p1 - p0, p2 - p1
        n0 = np.array([-e0[1], e0[0]]) / max(np.linalg.norm(e0), 1e-12)
        n1 = np.array([-e1[1], e1[0]]) / max(np.linalg.norm(e1), 1e-12)
        m = n0 + n1
        m /= max(np.linalg.norm(m), 1e-12)
        k = th / max(m @ n1, 0.3)
        inn.append(p1 + m * k)
    return build_prism(name, o2, [np.array(inn)], u, w, d, h0, h1)


def build_obb_box(name, me, shrink_mid=1.0):
    co, tri, _ = geo.mesh_arrays(me)
    ctr, ax = geo.pca_axes(co)
    p = (co - ctr) @ ax.T
    lo, hi = p.min(0), p.max(0)
    mid = (lo + hi) / 2
    lo[1] = mid[1] + (lo[1] - mid[1]) * shrink_mid
    hi[1] = mid[1] + (hi[1] - mid[1]) * shrink_mid
    verts = []
    for x in (lo[0], hi[0]):
        for y in (lo[1], hi[1]):
            for z in (lo[2], hi[2]):
                verts.append(ctr + ax.T @ np.array([x, y, z]))
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    me2 = new_mesh_from(name, [tuple(v) for v in verts], faces)
    fix_normals(me2)
    return me2



# ---------------------------------------------------------------- visibility of holes (static occluders + floor)

def _hemi_dirs(n):
    out = []
    for k in range(n):
        z = 1 - (k + 0.5) / n          # uniform in z over the upper hemisphere
        r = math.sqrt(max(0.0, 1 - z * z))
        th = k * math.pi * (3 - math.sqrt(5))
        out.append((r * math.cos(th), r * math.sin(th), z))
    return np.array(out)


HEMI = _hemi_dirs(32)
HEMI_DENSE = np.vstack([[(0.0, 0.0, 1.0)], _hemi_dirs(96)])  # exact normal first: slots seen only straight-on


def ray_escapes(scene, depsgraph, o, d, occluders, floor_z, max_dist=4.0):
    o = Vector(o)
    d = Vector(d).normalized()
    limit, floor_block = max_dist, False
    if d.z < -1e-6:
        tf = (o.z - floor_z) / -d.z
        if tf < limit:
            limit, floor_block = max(tf, 0.0), True
    p, remaining = o, limit
    for _ in range(10):
        if remaining <= 0:
            return not floor_block
        hit, loc, nrm, idx, obj, _m = scene.ray_cast(depsgraph, p, d, distance=remaining)
        if not hit:
            return not floor_block
        if obj is not None and obj.name in occluders:
            return False
        step = (loc - p).length + 2e-4
        remaining -= step
        p = loc + d * 2e-4
    return False


def point_visible(scene, depsgraph, origin_w, normal_w, occluders, floor_z, dense=False):
    n = np.asarray(normal_w, float)
    n /= np.linalg.norm(n)
    u, w = geo.perp_basis(n)
    for hx, hy, hz in (HEMI_DENSE if dense else HEMI):
        d = u * hx + w * hy + n * hz
        if ray_escapes(scene, depsgraph, origin_w, d, occluders, floor_z):
            return True
    return False


# ---------------------------------------------------------------- convex-hull prisms (hex shafts, hex spacers, standoffs)

def _hull2d(P):
    P = sorted(set(map(tuple, np.round(P, 7))))
    if len(P) < 3:
        return np.array(P)
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for p in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(P):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return np.array(lo[:-1] + hi[:-1])


def fit_hexagon(hull):
    """Regular hexagon fitted to a convex 2D outline: flats aligned with the outline's dominant edge directions."""
    H = np.asarray(hull)
    E = np.roll(H, -1, axis=0) - H
    ln = np.linalg.norm(E, axis=1)
    th = np.arctan2(E[:, 1], E[:, 0])
    z = (ln * np.exp(6j * th)).sum()
    phi = np.angle(z) / 6
    normals = np.array([[math.cos(phi + math.pi / 2 + k * math.pi / 3), math.sin(phi + math.pi / 2 + k * math.pi / 3)] for k in range(6)])
    c = H.mean(0)
    for _ in range(3):
        a = np.array([((H - c) @ n).max() for n in normals])
        shift = np.zeros(2)
        for k in range(3):
            shift += normals[k] * (a[k] - a[k + 3]) / 2
        c = c + shift * 2 / 3
    a = np.array([((H - c) @ n).max() for n in normals])
    ap = float(np.mean([(a[k] + a[k + 3]) / 2 for k in range(3)]))
    R = ap / math.cos(math.pi / 6)
    corners = [c + R * np.array([math.cos(phi + math.pi / 2 + math.pi / 6 + k * math.pi / 3),
                                 math.sin(phi + math.pi / 2 + math.pi / 6 + k * math.pi / 3)]) for k in range(6)]
    return np.array(corners)


def build_hull_prism(name, me, hexagon=False):
    """Best convex-hull (or exact hexagon) extrusion over the candidate axes; returns (mesh, p95 error) or (None, inf)."""
    co, tri, _ = geo.mesh_arrays(me)
    pts, _ = geo.sample_surface(co, tri, 4000)
    best = (None, 1e9)
    for d in geo.candidate_axes(pts):
        u, w = geo.perp_basis(d)
        h = co @ d
        hull = _hull2d(np.c_[co @ u, co @ w])
        if len(hull) < 3:
            continue
        if hexagon:
            hull = fit_hexagon(hull)
        else:
            hull = geo.douglas_peucker(hull, 0.3 * MM, closed=True)
        if len(hull) < 3:
            continue
        m = build_prism(name, hull, [], u, w, d, h.min(), h.max())
        e1, e2 = fit_error(me, m, 3000)
        err = max(e1, e2)
        if err < best[1]:
            if best[0] is not None:
                bpy.data.meshes.remove(best[0])
            best = (m, err)
        else:
            bpy.data.meshes.remove(m)
    return best



def hex_candidate(me, journals=True, bore_test=None):
    """Best axis for a hexagonal prism. Returns (mesh, dev_mm, apothem_mm) where dev is the max 2D gap
    between the part's outline and the fitted hexagon.
    bore_test(d, c, h0, h1, r) (cfg.HEX_THROUGH_BORE): a plain prism keeps the part's through-bore as an n-gon when
    the part is open along its axis and bore_test says nothing in the robot fills it (mesh["hex_bore_mm"] is set)."""
    co, tri, _ = geo.mesh_arrays(me)
    pts, _ = geo.sample_surface(co, tri, 4000)
    body_pts = geo.sample_surface(co, tri, 30000)[0] if HEX_BODY_FIT else None
    best = None
    for d in geo.candidate_axes(pts):
        u, w = geo.perp_basis(d)
        hull = _hull2d(np.c_[co @ u, co @ w]) if body_pts is None else _hull2d(_hex_body_2d(body_pts, d, u, w))
        if len(hull) < 3:
            continue
        hx = fit_hexagon(hull)
        c = hx.mean(0)
        ap = float(np.linalg.norm((hx[0] + hx[1]) / 2 - c))
        # distance of each hull point from the hexagon boundary
        nrm = []
        for i in range(6):
            mid = (hx[i] + hx[(i + 1) % 6]) / 2 - c
            nrm.append(mid / np.linalg.norm(mid))
        nrm = np.array(nrm)
        proj = (hull - c) @ nrm.T
        dev = float(np.abs(proj.max(1) - ap).max())
        # the outline must also reach the six corners (a round part touches the flats but not the corners)
        cdir = (hx - c) / np.linalg.norm(hx - c, axis=1)[:, None]
        reach = ((hull - c) @ cdir.T).max(0)
        dev = max(dev, float(np.abs(reach - ap / math.cos(math.pi / 6)).max()))
        h = co @ d
        if best is None or dev < best[1]:
            best = (d, dev, ap, hx, h.min(), h.max(), u, w)
    if best is None:
        return None, 1e9, 0
    d, dev, ap, hx, h0, h1, u, w = best
    holes = []
    rb = _hex_through_bore(co, tri, d, u, w, hx, ap, h0, h1) if bore_test is not None else None
    if rb and bore_test(d, u * hx.mean(0)[0] + w * hx.mean(0)[1], h0, h1, rb):  # centre without a d component
        c2 = hx.mean(0)
        n = geo.ngon_segments(rb / MM)
        ang = np.arange(n) * 2 * math.pi / n + math.pi / n
        holes = [np.c_[c2[0] + rb * np.cos(ang), c2[1] + rb * np.sin(ang)]]
    prism = (_hex_with_journals(co, d, u, w, hx, ap, h0, h1, holes=holes) if journals
             else build_prism("hex", hx, holes, u, w, d, h0, h1))
    if holes and prism.get("hex_bore_ok", True):
        prism["hex_bore_mm"] = 2 * rb / MM
    if "hex_bore_ok" in prism:
        del prism["hex_bore_ok"]
    return prism, dev / MM, ap / MM


def _hex_through_bore(co, tri, d, u, w, hx, ap, h0, h1):
    """Radius of a round through-bore along the hex axis, or None: vertices well inside the apothem form the bore wall,
    and rays along the axis at half that radius pass the part itself."""
    from mathutils.bvhtree import BVHTree
    c = hx.mean(0)
    r = np.hypot(co @ u - c[0], co @ w - c[1])
    inner = r[(r < 0.75 * ap) & (r > 0.5 * MM)]
    if len(inner) < 8:
        return None
    rb = float(np.median(inner))
    if rb < 1.0 * MM or rb > 0.7 * ap:
        return None
    bvh = BVHTree.FromPolygons([tuple(v) for v in co], [tuple(int(i) for i in t) for t in tri])
    L = (h1 - h0) + 2 * MM
    passed = 0
    for k in range(4):
        a = k * math.pi / 2 + 0.3
        p = Vector(u * (c[0] + 0.5 * rb * math.cos(a)) + w * (c[1] + 0.5 * rb * math.sin(a)) + d * (h0 - MM))
        if bvh.ray_cast(p, Vector(d), L)[0] is None:
            passed += 1
    return rb if passed >= 3 else None


# opt-in (cfg.HEX_BODY_FIT): fit the hexagon to the shaft BODY. The hull of all vertices takes in an E-clip or collar
# at the ends: a 1/2" rounded hex "Shaft" once came out 14.4 mm across flats (CAD 12.4) and 17.7 across corners (14.1).
HEX_BODY_FIT = False


def _hex_body_2d(pts, d, u, w, slice_mm=1.0, above=1.08):
    """2D points (u, w) of the surface samples in the body slices: slices whose radius (about the robust centre) stands
    more than `above` x the median slice radius (clips, collars, screw heads) are left out."""
    xy = np.c_[pts @ u, pts @ w]
    h = pts @ d
    ctr = np.median(xy, axis=0)
    r = np.linalg.norm(xy - ctr, axis=1)
    k = ((h - h.min()) / (slice_mm * MM)).astype(int)
    order = np.argsort(k)
    ks, rs = k[order], r[order]
    uk, start = np.unique(ks, return_index=True)
    rmax = np.maximum.reduceat(rs, start)
    keep = set(uk[rmax <= above * np.median(rmax)].tolist())
    sel = np.array([x in keep for x in k])
    return xy[sel] if sel.sum() >= 12 else xy


def _hex_with_journals(co, d, u, w, hx, ap, h0, h1, bin_mm=0.25, holes=()):
    """Hex prism over the hex length only; turned-down round ends (bearing journals) become 8-sided stubs.
    A full-length hex is wider than the bearing bore and ends flush on the bearing face (z-fighting)."""
    c = hx.mean(0)
    h = co @ d
    r = np.hypot(co @ u - c[0], co @ w - c[1])
    nb = max(int((h1 - h0) / (bin_mm * MM)), 1)
    k = np.clip(((h - h0) / max(h1 - h0, 1e-9) * nb).astype(int), 0, nb - 1)
    rmax = np.zeros(nb)
    np.maximum.at(rmax, k, r)
    # HEX_BODY_FIT: a slice is hex only if it reaches past the midpoint of apothem and corner radius (a round end turned to
    # the flat radius stays a round stub); default: 0.97 x apothem
    corner = float(np.linalg.norm(hx - c, axis=1).mean())
    is_hex = rmax > (0.5 * (ap + corner) if HEX_BODY_FIT else 0.97 * ap)
    if not is_hex.any():
        return build_prism("hex", hx, list(holes), u, w, d, h0, h1)
    i0, i1 = int(np.argmax(is_hex)), int(nb - 1 - np.argmax(is_hex[::-1]))
    hs0, hs1 = h0 + (h1 - h0) * i0 / nb, h0 + (h1 - h0) * (i1 + 1) / nb
    stubs = []
    for a, z in ((h0, hs0), (hs1, h1)):
        sel = (h >= a - 1e-9) & (h <= z + 1e-9)
        if z - a < 0.5 * MM or not sel.any():
            continue
        rs = float(r[sel].max())
        n = geo.ngon_segments(rs / MM)
        ang = np.arange(n) * 2 * math.pi / n
        stubs.append((np.c_[c[0] + rs * np.cos(ang), c[1] + rs * np.sin(ang)], a, z))
    if not stubs:
        return build_prism("hex", hx, list(holes), u, w, d, hs0, hs1)
    # a stub ending exactly on the hex end face puts its cap in the hex face's plane (z-fighting, a diamond mark on
    # nylon-insert lock nuts): every internal junction overlaps by JUNCTION_OVERLAP, both caps end up inside the neighbour
    e0 = JUNCTION_OVERLAP if any(a == h0 for _, a, _ in stubs) else 0.0
    e1 = JUNCTION_OVERLAP if any(z == h1 for _, _, z in stubs) else 0.0
    parts = [build_prism("hex", hx, [], u, w, d, hs0 - e0, hs1 + e1)]
    for ring, a, z in stubs:
        parts.append(build_prism("stub", ring, [], u, w, d, a, z + JUNCTION_OVERLAP) if a == h0 else
                     build_prism("stub", ring, [], u, w, d, a - JUNCTION_OVERLAP, z))
    verts, faces = [], []
    for m in parts:
        base = len(verts)
        verts += [tuple(v.co) for v in m.vertices]
        faces += [tuple(base + i for i in p.vertices) for p in m.polygons]
        bpy.data.meshes.remove(m)
    out = new_mesh_from("hex", verts, faces)
    if holes:
        out["hex_bore_ok"] = False  # a bore through hex + stubs is not built (stubs overlap the hex inside the bore)
    return out



# ---------------------------------------------------------------- long constant-profile parts (slides, channels)

def build_profile_extrusion(name, me):
    """Extrude the end-cap profile of a long part over its full length (side holes are dropped).
    Returns (mesh, p95 orig->new mm) or (None, inf)."""
    co, tri, _ = geo.mesh_arrays(me)
    pts, _ = geo.sample_surface(co, tri, 4000)
    _, ax = geo.pca_axes(pts)
    d = geo.snap_axis(ax[0])
    h = co @ d
    if np.ptp(h) < 6 * np.ptp(co - np.outer(h, d), axis=0).max():
        return None, 1e9
    u, w = geo.perp_basis(d)
    lv = level_loops(me, d, tol=1e-4)
    for key in sorted([k for k in lv if lv[k]], key=lambda k: -abs(k[0] - h.mean())):
        loops = [to2d(L, u, w) for L in lv[key]]
        ar = [abs(geo.poly_area(L)) for L in loops]
        if not ar or max(ar) <= 0:
            continue
        i0 = int(np.argmax(ar))
        outer = geo.douglas_peucker(loops[i0], 0.25 * MM, closed=True)
        holes = [geo.douglas_peucker(L, 0.25 * MM, closed=True) for i, L in enumerate(loops) if i != i0]
        holes = [H for H in holes if len(H) >= 3 and point_in_poly(H.mean(0), loops[i0])]
        m = build_prism(name, outer, holes, u, w, d, h.min(), h.max())
        e1, e2 = fit_error(me, m, 6000, q=99.5)
        return m, e1
    return None, 1e9


# ---------------------------------------------------------------- plug small holes before decimating

def plug_small_holes(me, max_diam=8e-3, sharp_deg=40, normals="recalc", keep=None, kept=None):
    """Remove small through-hole walls (closed tubular smooth patches with two round rims) and cap the rims,
    so Collapse decimation does not tear flat faces apart. Modifies the mesh in place; returns holes plugged.
    keep(rims) -> True leaves a hole open; rims = [(rim points (n, 3), outward normal of the surface around it)]."""
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.faces.ensure_lookup_table()
    cosl = math.cos(math.radians(sharp_deg))
    seen, patches = set(), []
    for f in bm.faces:
        if f.index in seen:
            continue
        stack, comp = [f], []
        seen.add(f.index)
        while stack:
            g = stack.pop()
            comp.append(g)
            for e in g.edges:
                if len(e.link_faces) != 2:
                    continue
                h = e.link_faces[0] if e.link_faces[1] is g else e.link_faces[1]
                if h.index not in seen and g.normal.dot(h.normal) > cosl:
                    seen.add(h.index)
                    stack.append(h)
        patches.append(comp)
    plugged = 0
    kill, rims = [], []
    for comp in patches:
        if len(comp) < 6 or len(comp) > 400:
            continue
        vs = {v for f in comp for v in f.verts}
        co = np.array([v.co[:] for v in vs])
        ext = np.ptp(co, axis=0)
        # size by cross-section, not the longest side: a 4 mm hole through a 12.7 mm boss is 12.7 mm long
        # (swerve base plates: 25 such holes per plate went unplugged and Collapse turned them into diamonds)
        if np.sort(ext)[1] > max_diam * 1.2:
            continue
        cset = set(comp)
        bnd = [e for f in comp for e in f.edges if sum(1 for g in e.link_faces if g in cset) == 1]
        # patch boundary must form exactly two closed loops (a tube)
        adj = {}
        for e in dict.fromkeys(bnd):  # ordered dedupe: a set iterates by address and changed caps run to run
            a, b = e.verts
            adj.setdefault(a, []).append(b)
            adj.setdefault(b, []).append(a)
        if not adj or any(len(n) != 2 for n in adj.values()):
            continue
        loops, used = [], set()
        for st in adj:
            if st in used:
                continue
            loop, prev, cur = [st], None, st
            used.add(st)
            while True:
                nx = [x for x in adj[cur] if x is not prev and x not in used]
                if not nx:
                    break
                prev, cur = cur, nx[0]
                used.add(cur)
                loop.append(cur)
            loops.append(loop)
        if len(loops) != 2 or min(len(l) for l in loops) < 4:
            continue
        # each rim roughly round and planar
        ok = True
        for loop in loops:
            p = np.array([v.co[:] for v in loop])
            c = p.mean(0)
            r = np.linalg.norm(p - c, axis=1)
            if r.mean() <= 0 or r.std() / r.mean() > 0.15 or 2 * r.mean() > max_diam * 1.05:
                ok = False
        if not ok:
            continue
        # a hole's wall faces its axis; a pin or boss (LED bump, dowel) has the same two round rims but faces out:
        # "plugging" one deleted its wall and left inside-out caps (beam-break sensor 9% back faces)
        c1 = np.array([v.co[:] for v in loops[0]]).mean(0)
        c2 = np.array([v.co[:] for v in loops[1]]).mean(0)
        ax = c2 - c1
        if np.linalg.norm(ax) > 1e-9:
            ax = ax / np.linalg.norm(ax)
            conc = 0.0
            for f in comp:
                fc = np.array(f.calc_center_median()) - c1
                rad = fc - (fc @ ax) * ax
                conc += f.calc_area() * float(np.sign(np.array(f.normal) @ rad))
            if conc > 0:
                continue
        if keep is not None:
            info = []
            for loop in loops:
                ring = set(loop)
                # the surface around the rim: faces on rim edges that are not hole wall
                outs = [g for v in loop for e in v.link_edges if e.other_vert(v) in ring
                        for g in e.link_faces if g not in cset]
                n = sum((g.normal * g.calc_area() for g in outs), Vector())
                if n.length > 0:
                    info.append((np.array([v.co[:] for v in loop]), np.array(n.normalized())))
            if keep(info):
                if kept is not None:  # wall mid-points of holes left open (rim points lie in the face plane, so
                    kept.extend(f.calc_center_median() for f in comp)  # a hole turned triangle still scored ~0)
                continue
        kill.extend(comp)
        rims.extend(loops)
        plugged += 1
    if plugged:
        bmesh.ops.delete(bm, geom=list(dict.fromkeys(kill)), context='FACES')
        bm.verts.ensure_lookup_table()
        cap_l = bm.faces.layers.int.new("__cap")
        for i, loop in enumerate(rims):
            if not all(v.is_valid for v in loop):
                continue
            try:
                f = bm.faces.new(loop)
            except (ValueError, TypeError):
                continue
            f[cap_l] = i + 1
            # the cap takes the colour of the surface around its rim, not slot 0 (a white cable gland's end
            # turned red on the Mag Limit Switch)
            ring = set(loop)
            mats = collections.Counter(g.material_index for v in loop for e in v.link_edges if e.other_vert(v) in ring
                                       for g in e.link_faces if g is not f)
            if mats:
                f.material_index = mats.most_common(1)[0][0]
        bmesh.ops.triangulate(bm, faces=[f for f in bm.faces if len(f.verts) > 4])
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
        {"parity": outward_normals, "vote": parity_vote_normals}.get(normals, consistent_normals)(bm)
        orient_caps(bm, cap_l)
        bm.faces.layers.int.remove(cap_l)
        flip_inverted_shells(bm)
        bm.to_mesh(me)
        me.update()
    bm.free()
    return plugged


def orient_caps(bm, cap_l):
    """A plug cap stands in for the surface the hole was cut through, so it faces the way the faces around its rim
    do. The normals pass can't decide that on unwelded CAD: WCP-0323's plugged holes came out facing inward
    (dark discs in Blender, see-through in Unity)."""
    groups = collections.defaultdict(list)
    for f in bm.faces:
        if f[cap_l]:
            groups[f[cap_l]].append(f)
    flipped = 0
    for fs in groups.values():
        fset = set(fs)
        around = {g for f in fs for e in f.edges for g in e.link_faces if g not in fset and not g[cap_l]}
        ns = sum((g.normal * g.calc_area() for g in around), Vector())
        nc = sum((f.normal * f.calc_area() for f in fs), Vector())
        if ns.length > 0 and nc.dot(ns) < 0:
            for f in fs:
                f.normal_flip()
            flipped += 1
    return flipped


def consistent_normals(bm):
    """Make winding consistent (planar dissolve and Collapse need it), then give each connected piece the
    orientation that agrees with its original normals. Without consistent winding the planar dissolve can't
    merge and Collapse leaves regions behind (error stuck at one value for every ratio)."""
    orig = {f: f.normal.copy() for f in bm.faces}
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    seen = set()
    for f0 in bm.faces:
        if f0 in seen:
            continue
        comp, stack = [], [f0]
        seen.add(f0)
        while stack:
            f = stack.pop()
            comp.append(f)
            for e in f.edges:
                for g in e.link_faces:
                    if g not in seen:
                        seen.add(g)
                        stack.append(g)
        agree = sum(f.calc_area() * (1 if f.normal.dot(orig[f]) > 0 else -1) for f in comp if f in orig)
        if agree < 0:
            for f in comp:
                f.normal_flip()


def flip_inverted_shells(bm):
    """Closed pieces with negative signed volume are inside-out (hole plugs left as separate shells, the Minion
    asset's shaft): flip them. Unity culls backfaces, so they render as dents or see-through pegs."""
    seen, flipped = set(), 0
    for f0 in bm.faces:
        if f0 in seen:
            continue
        comp, stack = [], [f0]
        seen.add(f0)
        while stack:
            f = stack.pop()
            comp.append(f)
            for e in f.edges:
                for g in e.link_faces:
                    if g not in seen:
                        seen.add(g)
                        stack.append(g)
        closed = all(len(e.link_faces) == 2 for f in comp for e in f.edges)
        if closed and sum(f.calc_center_median().dot(f.normal) * f.calc_area() for f in comp) < 0:
            for f in comp:
                f.normal_flip()
            flipped += 1
    return flipped


def fix_inverted_shells(me):
    bm = bmesh.new()
    bm.from_mesh(me)
    n = flip_inverted_shells(bm)
    if n:
        bm.to_mesh(me)
        me.update()
    bm.free()
    return n


def outward_normals(bm, rays=3):
    """Point every face outward by ray parity: from just off the face, along its normal, count crossings with the
    mesh; an odd count means the face looks inside. Planar dissolve and Collapse need consistent winding, and a
    plain recalc_face_normals turns non-manifold CAD partly inside-out (an encoder housing came out 30% flipped)."""
    from mathutils.bvhtree import BVHTree
    bm.faces.ensure_lookup_table()
    tree = BVHTree.FromBMesh(bm)
    span = max((v.co for v in bm.verts), key=lambda c: c.length).length * 4 + 1.0
    jit = [Vector((0, 0, 0)), Vector((0.07, 0.05, 0.0)), Vector((-0.05, 0.0, 0.06))]
    flip = []
    for f in bm.faces:
        n = f.normal
        if n.length < 0.5:
            continue
        inside = 0
        for j in jit[:rays]:
            dirv = (n + j).normalized()
            p = f.calc_center_median() + n * 1e-6
            hits, left = 0, span
            for _ in range(64):
                loc, nrm, idx, dist = tree.ray_cast(p, dirv, left)
                if loc is None:
                    break
                hits += 1
                left -= dist + 1e-6
                p = loc + dirv * 1e-6
            inside += hits % 2
        if inside * 2 > rays:
            flip.append(f)
    for f in flip:
        f.normal_flip()
    return len(flip)


def parity_vote_normals(bm, rays=3, max_faces=300):
    """Consistent winding (recalc) per manifold piece, then each piece faces the way most of its area says by ray
    parity. Per-face parity (outward_normals) left 250-480 inconsistent edges on the open, non-manifold encoder
    housings, and Collapse then folded them; recalc alone left one housing shell 26% inside-out."""
    from mathutils.bvhtree import BVHTree
    import random
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.faces.ensure_lookup_table()
    tree = BVHTree.FromBMesh(bm)
    span = max((v.co for v in bm.verts), key=lambda c: c.length).length * 4 + 1.0
    jit = [Vector((0, 0, 0)), Vector((0.07, 0.05, 0.0)), Vector((-0.05, 0.0, 0.06))]

    def looks_inside(f):
        n = f.normal
        inside = 0
        for j in jit[:rays]:
            dirv = (n + j).normalized()
            p = f.calc_center_median() + n * 1e-6
            hits, left = 0, span
            for _ in range(64):
                loc, nrm, idx, dist = tree.ray_cast(p, dirv, left)
                if loc is None:
                    break
                hits += 1
                left -= dist + 1e-6
                p = loc + dirv * 1e-6
            inside += hits % 2
        return inside * 2 > rays
    seen, flipped = set(), 0
    rng = random.Random(1)
    for f0 in bm.faces:
        if f0 in seen:
            continue
        comp, stack = [], [f0]
        seen.add(f0)
        while stack:
            f = stack.pop()
            comp.append(f)
            for e in f.edges:
                if len(e.link_faces) != 2:
                    continue  # pieces meet at non-manifold edges; recalc can't make winding agree across them
                for g in e.link_faces:
                    if g not in seen:
                        seen.add(g)
                        stack.append(g)
        sample = [f for f in comp if f.normal.length > 0.5]
        if len(sample) > max_faces:
            sample = rng.sample(sample, max_faces)
        score = sum(f.calc_area() * (-1 if looks_inside(f) else 1) for f in sample)
        if score < 0:
            for f in comp:
                f.normal_flip()
            flipped += 1
    return flipped


def cross_axis_holes(me, d, min_diam=3.5e-3, max_diam=20e-3):
    """Round through-holes (tubular patches with two round rims) whose axis is not along d. A stepped plate
    extruded along d cannot carry them: the stopbracket lost three 4.8 mm holes drilled across its length."""
    found = []

    def keep(rims):
        if len(rims) >= 2:
            p0, p1 = rims[0][0], rims[1][0]
            c0, c1 = p0.mean(0), p1.mean(0)
            ax = c1 - c0
            L = np.linalg.norm(ax)
            dia = 2 * float(np.linalg.norm(p0 - c0, axis=1).mean())
            if L > 1e-6 and min_diam <= dia and abs(float(ax @ np.asarray(d)) / L) < 0.9:
                found.append(dia)
        return True  # never modify
    tmp = me.copy()
    plug_small_holes(tmp, max_diam=max_diam, keep=keep)
    bpy.data.meshes.remove(tmp)
    return found


def protect_points(me, pts, name="keep_holes", tol=1e-6):
    """Vertex group (weight 1) on the vertices at pts, written into the mesh's deform data through a throwaway
    object. A Collapse Decimate pointed at it (high vertex_group_factor) collapses those vertices last, so the
    holes a part keeps stay round instead of turning into triangles (swerve plate WCP-1110-001)."""
    from mathutils.kdtree import KDTree
    if not pts or not len(me.vertices):
        return 0
    kd = KDTree(len(pts))
    for i, p in enumerate(pts):
        kd.insert(p, i)
    kd.balance()
    idx = [v.index for v in me.vertices if kd.find(v.co)[2] <= tol]
    if not idx:
        return 0
    tmp = bpy.data.objects.new("__vg", me)
    vg = tmp.vertex_groups.new(name=name)
    vg.add(idx, 1.0, 'REPLACE')
    bpy.data.objects.remove(tmp)
    return len(idx)


def round_hole_points(me, max_diam=80e-3):
    """Wall mid-points of every round through-hole up to max_diam (for the Collapse quality loop: a hole left open
    must stay round; the 1 mm p99 budget let a swerve plate's 10 mm holes become pentagons). Doesn't modify me."""
    pts = []
    tmp = me.copy()
    plug_small_holes(tmp, max_diam=max_diam, keep=lambda info: True, kept=pts)
    bpy.data.meshes.remove(tmp)
    return pts


def make_all_local():
    """Make every linked datablock local (materials, node groups, images, meshes, objects) and drop the library
    entries nothing uses any more. Returns how many datablocks were made local."""
    n = 0
    for _ in range(3):  # making one ID local can expose linked IDs it uses (node groups, images)
        linked = [i for coll in (bpy.data.objects, bpy.data.meshes, bpy.data.materials, bpy.data.node_groups,
                                 bpy.data.images, bpy.data.textures, bpy.data.collections)
                  for i in coll if i.library is not None]
        if not linked:
            break
        for i in linked:
            try:
                i.make_local()
                n += 1
            except RuntimeError:
                pass
    for lib in list(bpy.data.libraries):
        if lib.users == 0:
            bpy.data.libraries.remove(lib)
    return n


def backface_pct(me, grid=10):
    """Share (%) of first hits that land on a back face, for rays shot at the mesh from 14 directions. Unity culls
    back faces, so these show as see-through spots. Meaningful against the same part's CAD, not in absolute terms."""
    co_ = [v.co.copy() for v in me.vertices]
    if not co_ or not len(me.polygons):
        return 0.0
    t = BVHTree.FromPolygons(co_, [tuple(p.vertices) for p in me.polygons])
    c0 = sum(co_, Vector()) / len(co_)
    R0 = max((v - c0).length for v in co_) * 1.5 + 1e-6
    n_hit = n_back = 0
    for x in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1), (1, 1, 1), (-1, -1, 1),
              (1, -1, -1), (-1, 1, -1), (1, 1, 0), (0, 1, 1), (1, 0, 1), (-1, 0, 1)):
        dv = Vector(x).normalized()
        u_ = dv.orthogonal().normalized()
        w_ = dv.cross(u_).normalized()
        for i in range(-grid, grid + 1):
            for j in range(-grid, grid + 1):
                loc, nrm, idx, dist = t.ray_cast(c0 + dv * R0 + (u_ * i + w_ * j) * (R0 / grid), -dv)
                if loc is not None:
                    n_hit += 1
                    n_back += nrm.dot(-dv) > 0
    return 100.0 * n_back / max(n_hit, 1)


def prep_for_decimate(me, angle_deg=5.0, plug_max=8e-3, normals="recalc", keep=None, kept=None, weld_dist=None):
    """Plug small holes, then merge coplanar faces (shape-exact). Returns (holes plugged, tris after).
    weld_dist: Merge by Distance first (cfg.MERGE_DIST), so hole walls and rims are connected."""
    if weld_dist:
        weld(me, weld_dist, degenerate=True)
    backup = me.copy()
    done = []  # rims of the holes this pass plugs

    def keep_log(info):
        if keep is not None and keep(info):
            return True
        done.append(info)
        return False
    k = plug_small_holes(me, max_diam=plug_max, normals=normals, keep=keep_log, kept=kept)
    bf0 = backface_pct(backup) if k else 0.0
    if k and backface_pct(me) > bf0 + 0.5:
        # some plug broke the orientation (WCP-1113: 116 inconsistent edges; a 5x12.7 mm hole on a swerve plate).
        # Find the holes whose plug alone does it, leave those open and plug the rest; if that still fails (or there
        # are too many to test), leave every hole open.
        centres = [info[0][0].mean(0) for info in done if info]
        same = lambda info, c: bool(info) and np.linalg.norm(info[0][0].mean(0) - c) < 1e-6
        bad = []
        if len(centres) <= 80:
            for c in centres:
                t = backup.copy()
                plug_small_holes(t, max_diam=plug_max, normals=normals,
                                 keep=lambda info, c=c: (keep is not None and keep(info)) or not same(info, c))
                if backface_pct(t) > bf0 + 0.2:
                    bad.append(c)
                bpy.data.meshes.remove(t)
        bm = bmesh.new()
        bm.from_mesh(backup)
        bm.to_mesh(me)
        bm.free()
        me.update()
        k = 0
        if bad and len(bad) < len(centres):
            k = plug_small_holes(me, max_diam=plug_max, normals=normals, kept=kept,
                                 keep=lambda info: (keep is not None and keep(info)) or any(same(info, c) for c in bad))
            if backface_pct(me) > bf0 + 0.5:
                bm = bmesh.new()
                bm.from_mesh(backup)
                bm.to_mesh(me)
                bm.free()
                me.update()
                k = 0
    bpy.data.meshes.remove(backup)
    # broken CAD normals (Versahub 16%, x44 cover 17% of outside rays land on back faces): try the parity vote
    # and keep it when it is clearly better. Unity culls those faces, so the part shows see-through patches.
    bf = backface_pct(me)
    if bf > 2.0 and normals != "vote":
        alt = me.copy()
        bm = bmesh.new()
        bm.from_mesh(alt)
        parity_vote_normals(bm)
        bm.to_mesh(alt)
        bm.free()
        if backface_pct(alt) < bf - 1.0:
            bm = bmesh.new()
            bm.from_mesh(alt)
            bm.to_mesh(me)
            bm.free()
            me.update()
        bpy.data.meshes.remove(alt)
    elif normals == "vote" and not k:  # the plug pass (which applies the mode) did not run
        alt = me.copy()
        bm = bmesh.new()
        bm.from_mesh(alt)
        parity_vote_normals(bm)
        bm.to_mesh(alt)
        bm.free()
        # never accept a worse orientation: on the REV encoder housing the vote flipped the whole part (97%)
        if backface_pct(alt) < bf:
            bm = bmesh.new()
            bm.from_mesh(alt)
            bm.to_mesh(me)
            bm.free()
            me.update()
        bpy.data.meshes.remove(alt)
    fix_inverted_shells(me)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
    # delimit by material: merging coplanar faces across a colour border repaints one side (bumper lettering)
    bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(angle_deg), use_dissolve_boundaries=False,
                             verts=bm.verts, edges=bm.edges, delimit={'NORMAL', 'MATERIAL'})
    bm.to_mesh(me)
    bm.free()
    if "custom_normal" in me.attributes:
        me.attributes.remove(me.attributes["custom_normal"])
    me.update()
    return k, tri_count(me)


# ---------------------------------------------------------------- roller-chain links

def build_chain_link(name, me, pin_sides=6, outline_tol=0.5e-3):
    """Low-poly chain link: two side plates (real outline, holes dropped) + two pin/roller cylinders."""
    co, tri, _ = geo.mesh_arrays(me)
    a, b, c = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
    cr = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(cr, axis=1)
    nrm = cr / np.maximum(np.linalg.norm(cr, axis=1)[:, None], 1e-30)
    ctr, ax = geo.pca_axes(co)
    # plate normal = axis carrying the most flat area
    best = None
    for d in [geo.snap_axis(x) for x in ax]:
        par = np.abs(nrm @ d) > 0.99
        if best is None or area[par].sum() > best[1]:
            best = (d, area[par].sum())
    d = best[0]
    # refine to the true plate normal (link frames are often slightly rotated vs. the PCA axes)
    near = np.abs(nrm @ d) > 0.98
    if near.any():
        dd = (nrm[near] * (area[near] * np.sign(nrm[near] @ d))[:, None]).sum(0)
        d = geo.snap_axis(dd / np.linalg.norm(dd))
    rest = [x for x in ax if abs(x @ d) < 0.5]
    ext = [np.ptp(co @ r) for r in rest]
    u = rest[int(np.argmax(ext))]
    u = u - (u @ d) * d
    u = geo.snap_axis(u / np.linalg.norm(u))
    u = u - (u @ d) * d
    u /= np.linalg.norm(u)
    w = np.cross(d, u)
    hc = (co @ d).mean()
    fh = ((a + b + c) / 3) @ d
    sgn = np.sign(nrm @ d)
    par = np.abs(nrm @ d) > 0.99
    lv = collections.defaultdict(float)
    for h, s, ar in zip(np.round(fh[par] / 2e-5) * 2e-5, sgn[par], area[par]):
        lv[(round(float(h), 6), int(s))] += ar
    def pick(cond):
        cand = [(ar, k) for k, ar in lv.items() if cond(k)]
        return max(cand)[1][0] if cand else None
    top_o = pick(lambda k: k[1] > 0 and k[0] > hc)
    bot_o = pick(lambda k: k[1] < 0 and k[0] < hc)
    if top_o is None or bot_o is None:
        return None
    top_i = pick(lambda k: k[1] < 0 and hc < k[0] < top_o)
    bot_i = pick(lambda k: k[1] > 0 and bot_o < k[0] < hc)
    t_top = (top_o - top_i) if top_i is not None else 1.5e-3
    t_bot = (bot_i - bot_o) if bot_i is not None else 1.5e-3
    loops = level_loops(me, d, tol=1e-4, ignore_sign=True)
    if not loops:
        return None
    key = min(loops, key=lambda k: abs(k[0] - top_o))
    L2 = [to2d(L, u, w) for L in loops[key]]
    if not L2:
        return None
    outer = max(L2, key=lambda L: abs(geo.poly_area(L)))
    outline = geo.douglas_peucker(outer, outline_tol, closed=True)
    if len(outline) < 3:
        return None
    verts, faces = [], []

    def add_mesh(m2):
        base = len(verts)
        verts.extend([tuple(v.co) for v in m2.vertices])
        faces.extend([tuple(base + i for i in p.vertices) for p in m2.polygons])
        bpy.data.meshes.remove(m2)

    add_mesh(build_prism("p", outline, [], u, w, d, top_o - t_top, top_o))
    add_mesh(build_prism("p", outline, [], u, w, d, bot_o, bot_o + t_bot))
    # pin / roller centres: the two ends of the plate outline along u
    p2 = np.c_[co @ u, co @ w]
    hv = co @ d
    oc = outer.mean(0)
    span = np.ptp(outer[:, 0])
    hgt = np.ptp(outer[:, 1])
    for sgn_u in (-1, 1):
        cen = np.array([oc[0] + sgn_u * (span - hgt) / 2, oc[1]])
        between = (hv > bot_o + t_bot + 1e-4) & (hv < top_o - t_top - 1e-4)
        rr = np.linalg.norm(p2 - cen, axis=1)
        near = between & (rr < 0.6 * hgt)
        r = float(rr[near].max()) if near.any() else 0.25 * hgt
        inpin = rr < r + 2e-4
        h0, h1 = float(hv[inpin].min()), float(hv[inpin].max())
        ang = np.arange(pin_sides) * 2 * math.pi / pin_sides
        circ = np.c_[cen[0] + r * np.cos(ang), cen[1] + r * np.sin(ang)]
        add_mesh(build_prism("p", circ, [], u, w, d, h0, h1))
    out = new_mesh_from(name, verts, faces)
    return out


# ---------------------------------------------------------------- loose chain pieces (CAD exports with separate plates / pins)

def chain_piece_kind(me):
    """'plate' (thin side plate), 'pin' (round pin/roller) or 'link' (whole link in one mesh)."""
    co, tri, _ = geo.mesh_arrays(me)
    _, ax = geo.pca_axes(co)
    ext = np.sort([np.ptp(co @ a) for a in ax])
    if ext[0] < 1.5 * MM and ext[1] > 3 * MM:
        return "plate"
    pts, _ = geo.sample_surface(co, tri, 4000)
    if min(geo.revolve_score(pts, d)[0] for d in geo.candidate_axes(pts)) < 0.02:
        return "pin"
    return "link"


def build_chain_plate(name, me, end_pts=5):
    """Side plate, holes dropped: symmetric figure-8 rebuilt from the CAD outline. Each end is a semicircle
    (end_pts points: 5 = 45 deg steps, the 8-sided tier for r < 6 mm), plus the waist if it is visible."""
    co, tri, _ = geo.mesh_arrays(me)
    ctr, ax = geo.pca_axes(co)
    d = geo.snap_axis(ax[2])
    u = ax[0] - (ax[0] @ d) * d
    u /= np.linalg.norm(u)
    w = np.cross(d, u)
    h = co @ d
    loops = level_loops(me, d, tol=1e-4, ignore_sign=True)
    if not loops:
        return None
    key = max(loops, key=lambda k: k[0])
    if not loops[key]:
        return None
    outer = max((to2d(L, u, w) for L in loops[key]), key=lambda L: abs(geo.poly_area(L)))
    c2 = (outer.min(0) + outer.max(0)) / 2
    q = outer - c2
    half_len = np.abs(q[:, 0]).max()
    Re = np.abs(q[:, 1]).max()
    xc = max(half_len - Re, 0.0)
    mid = np.abs(q[:, 0]) < 0.12 * half_len
    Hw = float(np.abs(q[mid, 1]).min()) if mid.any() else Re
    pts = []
    for sgn in (1, -1):
        for k in range(end_pts):
            t = -math.pi / 2 + math.pi * k / (end_pts - 1)
            pts.append((sgn * (xc + Re * math.cos(t)), sgn * Re * math.sin(t)))
        if Re - Hw > 0.15 * MM:
            pts.append((0.0, sgn * Hw))
    P = np.array(pts) + c2
    return build_prism(name, P, [], u, w, d, h.min(), h.max())


def build_chain_pin(name, me, sides=6):
    """Pin/roller: one plain n-sided cylinder over its full length (largest radius)."""
    co, tri, _ = geo.mesh_arrays(me)
    pts, _ = geo.sample_surface(co, tri, 4000)
    best = min(((geo.revolve_score(pts, d), d) for d in geo.candidate_axes(pts)), key=lambda x: x[0][0])
    (_, c), d = best
    u, w = geo.perp_basis(d)
    rel = co - c
    h = rel @ d
    r = float(np.hypot(rel @ u, rel @ w).max())
    cc = np.array([c @ u, c @ w])
    ang = np.arange(sides) * 2 * math.pi / sides
    circ = np.c_[r * np.cos(ang), r * np.sin(ang)] + cc
    return build_prism(name, circ, [], u, w, d, h.min(), h.max())


# ---------------------------------------------------------------- coil springs

def _tube_along(path, radius, sides, closed=False):
    """Verts/faces of an n-sided tube swept along a polyline (parallel-transport frames). Open ends are capped."""
    P = np.asarray(path, float)
    n = len(P)
    T = np.gradient(P, axis=0) if not closed else (np.roll(P, -1, 0) - np.roll(P, 1, 0))
    T /= np.linalg.norm(T, axis=1)[:, None]
    a = np.cross(T[0], [0, 0, 1.0])
    if np.linalg.norm(a) < 1e-6:
        a = np.cross(T[0], [0, 1.0, 0])
    a /= np.linalg.norm(a)
    verts, faces, rings = [], [], []
    for i in range(n):
        if i:
            a = a - (a @ T[i]) * T[i]
            a /= np.linalg.norm(a)
        b = np.cross(T[i], a)
        ring = []
        for k in range(sides):
            t = 2 * math.pi * (k + 0.5) / sides
            ring.append(len(verts))
            verts.append(P[i] + radius * (a * math.cos(t) + b * math.sin(t)))
        rings.append(ring)
    segs = n if closed else n - 1
    for i in range(segs):
        A, B = rings[i], rings[(i + 1) % n]
        for k in range(sides):
            faces.append((A[k], A[(k + 1) % sides], B[(k + 1) % sides], B[k]))
    if not closed:
        faces.append(tuple(rings[0][::-1]))
        faces.append(tuple(rings[-1]))
    return verts, faces


def build_spring(name, me, seg_per_coil=12, wire_sides=4, hook_segs=10):
    """Coil spring with end loops: helix fitted to the CAD (axis, radius, pitch, phase, handedness) swept with an
    n-sided wire; each end loop is a circle fitted to the CAD points beyond the coil body."""
    co, tri, _ = geo.mesh_arrays(me)
    pts, _ = geo.sample_surface(co, tri, 60000)
    _, ax = geo.pca_axes(pts)
    d = geo.snap_axis(ax[0])
    u, w = geo.perp_basis(d)
    h = pts @ d
    pu, pw = pts @ u, pts @ w
    # body = middle 60%: centre from its bbox, outer radius from it
    hm0, hm1 = np.percentile(h, 20), np.percentile(h, 80)
    mid = (h > hm0) & (h < hm1)
    cu = 0.5 * (pu[mid].min() + pu[mid].max())
    cw = 0.5 * (pw[mid].min() + pw[mid].max())
    r = np.hypot(pu - cu, pw - cw)
    Ro = float(np.percentile(r[mid], 99.5))
    Ri = float(np.percentile(r[mid], 0.5))
    rw = (Ro - Ri) / 2  # wire radius: coil wall thickness is one wire diameter
    Rm = Ro - rw
    # body extent: where points still sit on the coil radius
    on = np.abs(r - Rm) < 1.6 * rw
    hb = h[on]
    th = np.arctan2(pw - cw, pu - cu)
    # pitch / handedness / phase: maximise the coherence of theta - s*2*pi*h/p over the body
    sel = mid & (r > Ro - 0.3 * rw)  # the wire's outermost line follows the helix exactly
    best = (0, None)
    for s in (1, -1):
        for p in np.linspace(2.05 * rw, (hm1 - hm0) / 2, 4000):
            z = np.exp(1j * (th[sel] - s * 2 * math.pi * h[sel] / p)).mean()
            if abs(z) > best[0]:
                best = (abs(z), (s, p, np.angle(z)))
    if best[1] is None or best[0] < 0.3:
        return None
    s, p, phi = best[1]
    print('SPRING', name, 'Ro', round(Ro * 1e3, 2), 'rw', round(rw * 1e3, 2), 'pitch', round(p * 1e3, 2), 'hand', s, 'coh', round(best[0], 2))
    # coil body: pitch-long slices whose wire points go all the way round the axis (end loops lie in
    # a plane through the axis, so their slices only cover two directions)
    ring = np.abs(r - Rm) < 1.5 * rw
    sec = ((th + math.pi) / (2 * math.pi) * 12).astype(int) % 12
    edges = np.arange(h.min(), h.max() + p, p)
    full = []
    for i in range(len(edges) - 1):
        m = ring & (h >= edges[i]) & (h < edges[i + 1])
        full.append(len(np.unique(sec[m])) >= 11)
    idx = [i for i, f in enumerate(full) if f]
    if len(idx) < 3:
        return None
    hb0, hb1 = edges[idx[0]], edges[idx[-1] + 1]
    h0, h1 = hb0 + rw, hb1 - rw
    nseg = max(8, int(round((h1 - h0) / p * seg_per_coil)))
    path = []
    for i in range(nseg + 1):
        hh = h0 + (h1 - h0) * i / nseg
        t = s * 2 * math.pi * hh / p + phi
        path.append(u * (cu + Rm * math.cos(t)) + w * (cw + Rm * math.sin(t)) + d * hh)
    V, F = _tube_along(path, rw, wire_sides)
    # end loops: points beyond the coil body, plane by PCA, circle fit (wire centre-line radius)
    for end in (-1, 1):
        m = (h < hb0) if end < 0 else (h > hb1)
        if m.sum() < 50:
            continue
        P = pts[m]
        c0, pax = geo.pca_axes(P)
        e1, e2 = pax[0], pax[1]
        q = np.c_[(P - c0) @ e1, (P - c0) @ e2]
        cc, rr, _ = geo.fit_circle(q)
        ring_pts = [c0 + e1 * (cc[0] + rr * math.cos(t)) + e2 * (cc[1] + rr * math.sin(t))
                    for t in np.arange(hook_segs) * 2 * math.pi / hook_segs]
        v2, f2 = _tube_along(ring_pts, rw, wire_sides, closed=True)
        b = len(V)
        V += v2
        F += [tuple(b + i for i in f) for f in f2]
    me2 = new_mesh_from(name, [tuple(v) for v in V], F, smooth_faces=set(range(len(F))))
    fix_normals(me2)
    return me2


# ---------------------------------------------------------------- wheels whose look is their cross-section (compliant wheels)

def extrude_loops(name, loops2d, u, w, d, h0, h1, test_loops=None, inside_fn=None):
    """Extrude a 2D region given as loops (even-odd rule: outline, windows, islands) from h0 to h1 along d.
    Walls follow the boundary of the kept cap triangles, so caps and walls always close up."""
    V, F = triangulate_region(loops2d, test_loops, inside_fn)
    verts, faces = [], []
    top = [len(verts) + i for i in range(len(V))]
    verts += [u * p[0] + w * p[1] + d * h1 for p in V]
    bot = [len(verts) + i for i in range(len(V))]
    verts += [u * p[0] + w * p[1] + d * h0 for p in V]
    count = collections.Counter()
    for f in F:
        a2, b2, c2 = V[f[0]], V[f[1]], V[f[2]]
        if (b2[0] - a2[0]) * (c2[1] - a2[1]) - (b2[1] - a2[1]) * (c2[0] - a2[0]) < 0:
            f = (f[0], f[2], f[1])  # CCW in 2D
        faces.append(tuple(top[i] for i in f))
        faces.append(tuple(bot[i] for i in f[::-1]))
        for i in range(3):
            count[frozenset((f[i], f[(i + 1) % 3]))] += 1
            count[(f[i], f[(i + 1) % 3])] += 0
    sharp, smooth = set(), set()
    directed = {}
    for f in F:
        a2, b2, c2 = V[f[0]], V[f[1]], V[f[2]]
        if (b2[0] - a2[0]) * (c2[1] - a2[1]) - (b2[1] - a2[1]) * (c2[0] - a2[0]) < 0:
            f = (f[0], f[2], f[1])
        for i in range(3):
            directed[frozenset((f[i], f[(i + 1) % 3]))] = (f[i], f[(i + 1) % 3])
    for key, n in count.items():
        if isinstance(key, frozenset) and n == 1:
            i, j = directed[key]  # boundary edge, CCW along the material: wall faces outward to its right
            smooth.add(len(faces))
            faces.append((top[j], top[i], bot[i], bot[j]))
            sharp.add(frozenset((top[i], top[j])))
            sharp.add(frozenset((bot[i], bot[j])))
    me = new_mesh_from(name, [tuple(v) for v in verts], faces, smooth_faces=smooth, sharp_edges=sharp)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context='VERTS')
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    me.update()
    return me


def section_loops(me, d, h, join_tol=3e-3):
    """Closed 2D loops (u, w coords) of the mesh cut by the plane h along d. CAD parts are often several
    touching, unwelded shells, so the cut has open chains: join them end-to-end (nearest endpoint first)
    before closing loops; closing each chain on itself made wrong outlines that filled wheel windows."""
    u, w = geo.perp_basis(d)
    bm = bmesh.new()
    bm.from_mesh(me)
    res = bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], dist=1e-7,
                                 plane_co=Vector(d * h), plane_no=Vector(d))
    cut = [e for e in res["geom_cut"] if isinstance(e, bmesh.types.BMEdge)]
    adj = collections.defaultdict(list)
    for e in cut:
        a, b2 = e.verts
        adj[a].append(b2)
        adj[b2].append(a)
    P = {v: np.array([v.co.dot(Vector(u)), v.co.dot(Vector(w))]) for v in adj}
    used, paths, loops = set(), [], []

    def walk(st):
        path, prev, cur = [st], None, st
        used.add(st)
        while True:
            nx = [x for x in adj[cur] if x is not prev and x not in used]
            if not nx:
                return path, cur in adj[st] and len(path) > 2
            prev, cur = cur, nx[0]
            used.add(cur)
            path.append(cur)
    for st in [v for v in adj if len(adj[v]) == 1]:   # open chains first, from their ends
        if st not in used:
            paths.append(walk(st)[0])
    for st in list(adj):                                # then closed cycles
        if st not in used:
            pth, _ = walk(st)
            if len(pth) >= 3:
                loops.append(np.array([P[v] for v in pth]))
    chains = [np.array([P[v] for v in pth]) for pth in paths if len(pth) >= 2]
    # global matching: join the closest free endpoint pairs first (greedy nearest-from-here picked wrong partners)
    ends = [(i, k) for i in range(len(chains)) for k in (0, 1)]
    pt = lambda e: chains[e[0]][0 if e[1] == 0 else -1]
    pairs = sorted((float(np.linalg.norm(pt(a) - pt(c))), a, c) for x, a in enumerate(ends) for c in ends[x + 1:])
    mate = {}
    for g, a, c in pairs:
        if g > join_tol:
            break
        if a not in mate and c not in mate:
            mate[a], mate[c] = c, a
    done = set()
    for i in range(len(chains)):
        if i in done:
            continue
        seq, ci, k = [], i, 0  # enter chain ci at end k
        while ci not in done:
            done.add(ci)
            seq.append(chains[ci] if k == 0 else chains[ci][::-1])
            out = (ci, 1 - k)
            if out not in mate:
                break
            ci, k = mate[out]
        L = np.vstack(seq)
        if len(L) >= 3:
            loops.append(L)
    bm.free()
    return loops, u, w


def repair_window_pattern(loops, area_tol=0.03, rad_tol=1.0 * MM, ang_tol=math.radians(2)):
    """Bad STEP tessellation can fan triangles across one window of a wheel, so its section comes out as
    fragments and the window fills in. Windows are a circular pattern: find the repeated window (same area and
    radius), and put a rotated copy of it into each empty slot, dropping whatever fragments sat there.
    Returns the loops unchanged unless the pattern is unambiguous."""
    if len(loops) < 4:
        return loops
    outer = max(loops, key=lambda L: abs(geo.poly_area(L)))
    c = (outer.max(0) + outer.min(0)) / 2
    info = []
    for i, L in enumerate(loops):
        m = L.mean(0)
        info.append((i, abs(geo.poly_area(L)), float(np.linalg.norm(m - c)), math.atan2(m[1] - c[1], m[0] - c[0])))
    best = []
    for i, a, r, _ in info:
        if r < 2 * MM:
            continue
        grp = [x for x in info if abs(x[1] - a) < area_tol * a and abs(x[2] - r) < rad_tol]
        if len(grp) > len(best):
            best = grp
    if len(best) < 3:
        return loops
    ang = sorted(x[3] % (2 * math.pi) for x in best)
    gaps_ = [(ang[(k + 1) % len(ang)] - ang[k]) % (2 * math.pi) for k in range(len(ang))]
    step = min(gaps_)
    n = round(2 * math.pi / step)
    if n <= len(best) or abs(n * step - 2 * math.pi) > n * ang_tol:
        return loops
    step = 2 * math.pi / n
    t = best[0]
    slots = [t[3] + k * step for k in range(n)]
    off = lambda a, s: abs((a - s + math.pi) % (2 * math.pi) - math.pi)
    if any(min(off(x[3], s) for s in slots) > ang_tol for x in best):
        return loops
    tmpl = loops[t[0]] - c
    out, drop = list(loops), set()
    for s in slots:
        if any(off(x[3], s) < ang_tol for x in best):
            continue
        q = s - t[3]
        R = np.array([[math.cos(q), -math.sin(q)], [math.sin(q), math.cos(q)]])
        new = tmpl @ R.T + c
        for i, a, r, _ in info:  # fragments: a centroid or most of their outline inside the new window
            if i in drop or loops[i] is outer or a > 1.2 * t[1]:
                continue
            inside = parity_inside(loops[i], [new]).mean() > 0.5
            if inside or parity_inside(loops[i].mean(0)[None], [new])[0]:
                drop.add(i)
        out.append(new)
    return [L for i, L in enumerate(out) if i not in drop]


def build_wheel_section(name, me, tol_mm=0.5, circle_sides=geo.ngon_segments):
    """Wheel whose spokes/fins are its look: mid-width cross-section (outline, windows, fins), simplified and
    extruded over the full width. Circles get the radius side-count tiers. Returns (mesh, fit errors) or (None, None)."""
    co, tri, _ = geo.mesh_arrays(me)
    pts, _ = geo.sample_surface(co, tri, 20000)
    best = min(((geo.revolve_score(pts, d), d) for d in geo.candidate_axes(pts)), key=lambda x: x[0][0])
    d = best[1]
    h = co @ d

    loops, u, w = section_loops(me, d, (h.min() + h.max()) / 2)
    loops = [L for L in loops if abs(geo.poly_area(L)) > (0.3 * MM) ** 2]
    # touching, unwelded shells (core + overmold) can both cut the same window outline: two copies cancel under
    # the even-odd rule and the window fills in, so keep one of each near-duplicate loop
    uniq = []
    for L in sorted(loops, key=lambda L: -abs(geo.poly_area(L))):
        a, c = abs(geo.poly_area(L)), L.mean(0)
        if not any(abs(a - abs(geo.poly_area(M))) < 0.02 * a and np.linalg.norm(c - M.mean(0)) < 0.3 * MM for M in uniq):
            uniq.append(L)
    loops = repair_window_pattern(uniq)
    if not loops:
        return None, None
    simp = []
    for L in loops:
        s = _simplify_loop(_dedupe_loop(L), tol_mm * MM, circle_sides)
        if s is not None and len(s) >= 3:
            simp.append(np.asarray(s))
    if not simp:
        return None, None
    # inside/outside from the unsimplified section: a simplified window can overlap the simplified hub or rim
    # polygon, and the even-odd test on overlapping loops filled whole windows
    new = extrude_loops(name, simp, u, w, d, h.min(), h.max(), test_loops=loops)
    return new, fit_error(me, new)


# ---------------------------------------------------------------- box tubes rotated inside their own mesh frame

def tube_frame(me):
    """Rows (long axis, wall normal A, wall normal B) that make a box tube axis-aligned, or None if it already is.
    Long axis from PCA; wall normals from the 4-fold average of the side-face normals (a 1x1 tube's PCA is ambiguous)."""
    co, tri, _ = geo.mesh_arrays(me)
    a, b, c = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
    cr = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(cr, axis=1)
    nrm = cr / np.maximum(np.linalg.norm(cr, axis=1)[:, None], 1e-30)
    pts, _ = geo.sample_surface(co, tri, 6000)
    _, ax = geo.pca_axes(pts)
    L = geo.snap_axis(ax[0])
    side = np.abs(nrm @ L) < 0.1
    if not side.any():
        return None
    u, w = geo.perp_basis(L)
    th = np.arctan2(nrm[side] @ w, nrm[side] @ u)
    phi = np.angle((area[side] * np.exp(4j * th)).sum()) / 4
    A = geo.snap_axis(u * math.cos(phi) + w * math.sin(phi))
    A = A - (A @ L) * L
    A /= np.linalg.norm(A)
    if TUBE_MITRED:
        # the PCA long axis of a mitred tube with dense holes is ~0.2 deg off: refine it as the direction normal
        # to both wall-normal families (area-weighted mean normals of the four walls), so the walls come out exact
        B = np.cross(L, A)
        fam = []
        for D in (A, B):
            sel = np.abs(nrm @ D) > 0.99
            if not sel.any():
                break
            fam.append((area[sel, None] * nrm[sel] * np.sign(nrm[sel] @ D)[:, None]).sum(0))
        if len(fam) == 2:
            L2 = np.cross(fam[0], fam[1])
            L2 /= np.linalg.norm(L2)
            L2 *= np.sign(L2 @ L)
            if L2 @ L > math.cos(math.radians(2)):
                L = L2
                A = fam[0] - (fam[0] @ L) * L
                A /= np.linalg.norm(A)
    R = np.array([L, A, np.cross(L, A)])
    if all(np.abs(r).max() > 0.9999 for r in R):
        return None
    return R


def frame_dims(me, R=None):
    """Extents (m) of the mesh in frame R (rows), or in its own axes."""
    co, _, _ = geo.mesh_arrays(me)
    return np.ptp(co @ R.T if R is not None else co, axis=0)


class Framed:
    """Stand-in for an object whose mesh was re-expressed in another frame: the ray helpers only need
    .name and .matrix_world (object matrix times the frame-to-mesh rotation)."""
    def __init__(self, obj, frame_to_mesh):
        self.name = obj.name
        self.matrix_world = obj.matrix_world @ frame_to_mesh


def largest_piece(objs):
    """The piece a multi-piece library swap lands on (pipeline and renders must agree): biggest world bbox diagonal."""
    def diag(o):
        bb = [o.matrix_world @ Vector(c) for c in o.bound_box]
        mn = Vector([min(v[i] for v in bb) for i in range(3)])
        mx = Vector([max(v[i] for v in bb) for i in range(3)])
        return (mx - mn).length
    return max(objs, key=lambda o: (diag(o), o.name))


# ---------------------------------------------------------------- eyebolts

def build_eyebolt(name, me, shank_sides=8, eye_segs=12, wire_sides=6):
    """Eyebolt: plain shank cylinder (threads dropped) + low-poly torus eye fitted to the CAD.
    Decimating the threads gave lumpy eyes and spiky threads at ~1.5-2k tris."""
    co, tri, _ = geo.mesh_arrays(me)
    pts, _ = geo.sample_surface(co, tri, 40000)
    _, ax = geo.pca_axes(pts)
    d = geo.snap_axis(ax[0])
    u, w = geo.perp_basis(d)
    h = pts @ d
    # orient d from the shank tip towards the eye: the eye end is the wide one
    lo, hi = np.percentile(h, 10), np.percentile(h, 90)
    wid = lambda m: np.ptp(pts[m] @ u) + np.ptp(pts[m] @ w)
    if wid(h < lo) > wid(h > hi):
        d, h = -d, -h
        u, w = geo.perp_basis(d)
    pu, pw = pts @ u, pts @ w
    low = h < np.percentile(h, 40)
    cu, cw = np.median(pu[low]), np.median(pw[low])
    r = np.hypot(pu - cu, pw - cw)
    rs = float(np.percentile(r[low], 95))
    # eye region: everything clearly wider than the shank
    wide = r > 1.8 * rs
    if wide.sum() < 200:
        return None
    he0 = float(np.percentile(h[wide], 1))
    E = pts[h > he0 - rs]
    c0, pax = geo.pca_axes(E)
    e1, e2 = pax[0], pax[1]
    q = np.c_[(E - c0) @ e1, (E - c0) @ e2]
    cc, R, _ = geo.fit_circle(q)
    rr = np.linalg.norm(q - cc, axis=1)
    rw = float(max((np.percentile(rr, 97) - np.percentile(rr, 3)) / 2, rs * 0.5))
    Rc = float((np.percentile(rr, 97) + np.percentile(rr, 3)) / 2)
    centre = c0 + e1 * cc[0] + e2 * cc[1]
    ring = [centre + e1 * Rc * math.cos(t) + e2 * Rc * math.sin(t) for t in np.arange(eye_segs) * 2 * math.pi / eye_segs]
    V, F = _tube_along(ring, rw, wire_sides, closed=True)
    # shank: from the tip to where it meets the eye ring
    h_end = float(centre @ d - Rc)
    ang = np.arange(shank_sides) * 2 * math.pi / shank_sides
    circ = np.c_[cu + rs * np.cos(ang), cw + rs * np.sin(ang)]
    sh = build_prism("s", circ, [], u, w, d, float(h.min()), h_end + rw * 0.5)
    smooth = set(range(len(F)))  # torus
    b0 = len(V)
    V += [tuple(v.co) for v in sh.vertices]
    sharp = set()
    for p in sh.polygons:
        if p.use_smooth:  # shank sides smooth, end caps flat with sharp rims (shared vertices shaded it as a cone)
            smooth.add(len(F))
        else:
            n = len(p.vertices)
            sharp.update(frozenset((b0 + p.vertices[i], b0 + p.vertices[(i + 1) % n])) for i in range(n))
        F.append(tuple(b0 + i for i in p.vertices))
    bpy.data.meshes.remove(sh)
    me2 = new_mesh_from(name, [tuple(v) for v in V], F, smooth_faces=smooth, sharp_edges=sharp)
    fix_normals(me2)
    return me2


# ---------------------------------------------------------------- weld, tube walls, cups, mixed shafts, kraken cap

def weld(me, dist, degenerate=False):
    """Merge by Distance on mesh data (bmesh keeps custom normals per corner). Returns verts removed.
    glTF exports can be unwelded polygon soup, which breaks everything that walks edges."""
    bm = bmesh.new()
    bm.from_mesh(me)
    n0 = len(bm.verts)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=dist)
    if degenerate:
        bmesh.ops.dissolve_degenerate(bm, edges=bm.edges, dist=dist)
    n = n0 - len(bm.verts)
    if n:
        bm.to_mesh(me)
        me.update()
    bm.free()
    return n


def _tube_walls_from_sides(co, tri, mn, mx, dims, others):
    """Wall thickness per cross-section axis from the largest inward-facing face level near each outer face
    (the inner wall surface). Needs all four walls; returns None for channels or solid bars."""
    a, b, c = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
    cr = np.cross(b - a, c - a)
    ar = 0.5 * np.linalg.norm(cr, axis=1)
    nrm = cr / np.maximum(np.linalg.norm(cr, axis=1)[:, None], 1e-30)
    cen = (a + b + c) / 3
    out = []
    for k in others:
        lim = 0.3 * dims[k]
        ts = []
        for s, outer in ((1, mx[k]), (-1, mn[k])):
            sel = (nrm[:, k] * -s > 0.999) & (np.abs(cen[:, k] - outer) < lim) & (np.abs(cen[:, k] - outer) > 1e-5)
            if not sel.any():
                return None
            lv = np.round(cen[sel, k] / 2e-5)
            uq, inv = np.unique(lv, return_inverse=True)
            area = np.bincount(inv, weights=ar[sel])
            if area.max() < 0.1 * dims[k] * dims.max():  # a real wall face runs most of the length
                return None
            ts.append(abs(outer - uq[int(np.argmax(area))] * 2e-5))
        out.append(min(ts))
    return np.array(out)


def build_cap_prism(name, me, arc_tol=0.15 * MM):
    """Low-poly cover (Kraken connector cap): the part's convex outline seen along its thinnest local axis,
    extruded through its depth. arc_tol 0.15 mm keeps a 30 mm radius arc at about the 32-sided spacing."""
    co, _, _ = geo.mesh_arrays(me)
    k = int(np.argmin(np.ptp(co, axis=0)))
    d = np.eye(3)[k]
    u, w = geo.perp_basis(d)
    hull = geo.douglas_peucker(_hull2d(np.c_[co @ u, co @ w]), arc_tol, closed=True)
    h = co @ d
    return build_prism(name, hull, [], u, w, d, h.min(), h.max())


def revolve_loop(name, prof, d, centre, n):
    """Revolve a closed (h, r) profile loop about axis d through centre with n sides; r == 0 points are poles."""
    u, w = geo.perp_basis(d)
    verts, faces = [], []
    ring = []  # per profile point: list of vertex indices (len 1 if on axis)
    eps = 1e-6
    for (h, r) in prof:
        base = centre + d * h
        if r < eps:
            ring.append([len(verts)])
            verts.append(base)
        else:
            idx = []
            for k in range(n):
                a = 2 * math.pi * k / n
                idx.append(len(verts))
                verts.append(base + (u * math.cos(a) + w * math.sin(a)) * r)
            ring.append(idx)
    smooth, sharp = set(), set()
    fseg = []
    m = len(prof)
    # turning angle at each profile vertex (for sharp ring edges)
    def turn(i):
        p0, p1, p2 = np.array(prof[i - 1]), np.array(prof[i]), np.array(prof[(i + 1) % m])
        a, b = p1 - p0, p2 - p1
        na, nb_ = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-9 or nb_ < 1e-9:
            return 180
        return math.degrees(math.acos(np.clip(a @ b / (na * nb_), -1, 1)))
    for i in range(m):
        j = (i + 1) % m
        A, B = ring[i], ring[j]
        if len(A) == 1 and len(B) == 1:
            continue
        # a segment is a "side" (smooth) if it is not perpendicular to the axis
        dh = abs(prof[j][0] - prof[i][0])
        dr = abs(prof[j][1] - prof[i][1])
        is_side = dh > 0.2 * dr
        for k in range(n):
            k2 = (k + 1) % n
            if len(A) == 1:
                f = (A[0], B[k], B[k2])
            elif len(B) == 1:
                f = (A[k], A[k2], B[0])
            else:
                f = (A[k], A[k2], B[k2], B[k])
            if is_side:
                smooth.add(len(faces))
            fseg.append(i)
            faces.append(f)
    for i in range(m):
        if len(ring[i]) > 1 and turn(i) > 30:
            R = ring[i]
            for k in range(n):
                sharp.add(frozenset((R[k], R[(k + 1) % n])))
    me = new_mesh_from(name, [tuple(v) for v in verts], faces, smooth, sharp)
    if len(fseg) == len(me.polygons):  # degenerate faces dropped by new_mesh_from -> no ring materials
        a = me.attributes.new("seg", 'INT', 'FACE')
        a.data.foreach_set("value", fseg)
    fix_normals(me)
    return me


def section_profile(me, d, centre, nang=24, res=None):
    """Meridional profile of a part as the union of its cross-sections through the axis at nang angles
    (spoke windows and tooth gaps fill in, open cavities such as cups stay open). Returns (h, r) loops."""
    co, tri, _ = geo.mesh_arrays(me)
    u, w = geo.perp_basis(d)
    rel = co - centre
    H = rel @ d
    rmax = float(np.linalg.norm(rel - np.outer(H, d), axis=1).max())
    h0, h1 = float(H.min()), float(H.max())
    res = res or max(0.05 * MM, min(0.2 * MM, max(rmax, h1 - h0) / 400))
    nr, nh = int(rmax / res) + 3, int((h1 - h0) / res) + 3
    rows = h0 - res + (np.arange(nh) + 0.5) * res
    cols = (np.arange(nr) + 0.5) * res
    union = np.zeros((nh, nr), bool)
    areas, fills = [], []
    for k in range(nang):
        th = 2 * math.pi * k / nang
        e = u * math.cos(th) + w * math.sin(th)
        nrm = -u * math.sin(th) + w * math.cos(th)
        sd = rel @ nrm
        A, B, C = tri[:, 0], tri[:, 1], tri[:, 2]
        # triangle/plane intersections: each crossing triangle gives one segment between its two crossing edges
        s0, s1, s2 = sd[A], sd[B], sd[C]
        cross = (np.sign(s0) != np.sign(s1)) | (np.sign(s1) != np.sign(s2))
        P = []
        for (ia, ib, sa, sb) in ((A, B, s0, s1), (B, C, s1, s2), (C, A, s2, s0)):
            t = np.where(np.abs(sa - sb) > 1e-15, sa / np.where(np.abs(sa - sb) > 1e-15, sa - sb, 1), 0.5)
            pt = rel[ia] + (rel[ib] - rel[ia]) * t[:, None]
            ok = (np.sign(sa) != np.sign(sb)) & cross
            P.append((pt, ok))
        ends = []
        for (pa, oka), (pb, okb) in ((P[0], P[1]), (P[1], P[2]), (P[2], P[0])):
            m = oka & okb
            if m.any():
                ends.append((pa[m], pb[m]))
        if not ends:
            continue
        Pa = np.vstack([x[0] for x in ends])
        Pb = np.vstack([x[1] for x in ends])
        ha, hb = Pa @ d, Pb @ d
        ra, rb = Pa @ e, Pb @ e
        keep = (ra > 0) | (rb > 0)
        ha, hb, ra, rb = ha[keep], hb[keep], np.maximum(ra[keep], 0), np.maximum(rb[keep], 0)
        g = np.zeros((nh, nr), bool)
        for j, hr in enumerate(rows):
            m = (ha - hr) * (hb - hr) < 0
            if not m.any():
                continue
            t = (hr - ha[m]) / (hb[m] - ha[m])
            rr = np.sort(ra[m] + (rb[m] - ra[m]) * t)
            for q in range(0, len(rr) - 1, 2):
                g[j, (cols >= rr[q]) & (cols <= rr[q + 1])] = True
        areas.append(g.sum())
        fills.append(g)
    if not fills:
        return []
    med = np.median(areas)
    for g, a in zip(fills, areas):
        if 0.3 * med <= a <= 3 * med:  # a broken (non-closed) section fills streaks; skip it
            union |= g
    # fill enclosed voids (invisible): keep only empty cells reachable from the grid border
    empty = ~union
    reach = np.zeros_like(empty)
    stack = [(j, i) for j in range(nh) for i in (0, nr - 1)] + [(j, i) for j in (0, nh - 1) for i in range(nr)]
    stack = [(j, i) for j, i in stack if empty[j, i]]
    for j, i in stack:
        reach[j, i] = True
    while stack:
        j, i = stack.pop()
        for jj, ii in ((j + 1, i), (j - 1, i), (j, i + 1), (j, i - 1)):
            if 0 <= jj < nh and 0 <= ii < nr and empty[jj, ii] and not reach[jj, ii]:
                reach[jj, ii] = True
                stack.append((jj, ii))
    # the axis column is not "outside": cells left of the first column count as material side
    solid = ~reach
    # trace boundary edges of solid cells (grid corners), r = 0 at column edge 0
    edges = {}
    def add(a, b):
        edges.setdefault(a, []).append(b)
    for j in range(nh):
        for i in range(nr):
            if not solid[j, i]:
                continue
            if i == 0 or not solid[j, i - 1]:
                add((j + 1, i), (j, i))
            if i == nr - 1 or not solid[j, i + 1]:
                add((j, i + 1), (j + 1, i + 1))
            if j == 0 or not solid[j - 1, i]:
                add((j, i), (j, i + 1))
            if j == nh - 1 or not solid[j + 1, i]:
                add((j + 1, i + 1), (j + 1, i))
    loops = []
    while edges:
        start = next(iter(edges))
        loop, cur = [start], start
        while True:
            nxt = edges[cur].pop()
            if not edges[cur]:
                del edges[cur]
            if nxt == start:
                break
            loop.append(nxt)
            cur = nxt
            if cur not in edges:
                break
        if len(loop) >= 4:
            P = np.array([(h0 - res + j * res, i * res) for j, i in loop])
            loops.append(P)
    return loops


def build_revolve_sections(name, src_me, d, centre, nseg=None, tol_mm=None):
    """Lathe rebuild from the union of axial cross-sections (keeps cups and stepped bores that are open)."""
    loops = section_profile(src_me, d, centre)
    if not loops:
        return None
    rmax = max(float(L[:, 1].max()) for L in loops)
    tol = (tol_mm * MM) if tol_mm else max(0.15 * MM, 0.03 * rmax)  # same 3% as build_revolve
    n = nseg or geo.ngon_segments(rmax / MM)
    parts = []
    for L in loops:
        if abs(geo.poly_area(L)) < (2 * tol) ** 2:
            continue
        S = geo.douglas_peucker(L, tol, closed=True)
        S[:, 1] = np.where(S[:, 1] < 0.6 * tol, 0.0, S[:, 1])
        prof = []
        for q in map(tuple, S):
            if not prof or abs(q[0] - prof[-1][0]) > 1e-7 or abs(q[1] - prof[-1][1]) > 1e-7:
                prof.append(q)
        if len(prof) > 2 and abs(prof[0][0] - prof[-1][0]) < 1e-7 and abs(prof[0][1] - prof[-1][1]) < 1e-7:
            prof.pop()
        if len(prof) < 3:
            continue
        parts.append(revolve_loop(name, prof, d, centre, n))
    if not parts:
        return None
    if len(parts) == 1:
        return parts[0]
    bm = bmesh.new()  # several separate rings: join, keeping smooth flags, sharp edges and the "seg" layer
    for m in parts:
        bm.from_mesh(m)
    bm.to_mesh(parts[0])
    bm.free()
    for m in parts[1:]:
        bpy.data.meshes.remove(m)
    parts[0].update()
    return parts[0]


def build_head_cylinder(name, src_me, d, centre):
    """(owner: "make rivets just one cylinder for the rivet head") One closed cylinder for the head of a rivet or
    bolt: the widest band of the revolve profile (radius above halfway between the shank and the head), extended to the
    part's end on that side so a dome head is covered, at the owner's side count for its radius. Shank and tail dropped."""
    co, tri, _ = geo.mesh_arrays(src_me)
    edges, Ro, _ = revolve_profile(co, tri, d, centre)
    k = int(np.argmax(Ro))
    Rmax, Rs = float(Ro[k]), float(np.median(Ro))
    mid = 0.5 * (Rmax + Rs)
    a = k
    while a > 0 and Ro[a - 1] >= mid:
        a -= 1
    b = k
    while b < len(Ro) - 1 and Ro[b + 1] >= mid:
        b += 1
    # the head sits at one end: extend to the nearer end of the part (covers a dome or a chamfered top)
    if a < len(Ro) - 1 - b:
        h0, h1 = edges[0], edges[b + 1]
    else:
        h0, h1 = edges[a], edges[-1]
    if h1 - h0 < 0.2 * MM or Rmax <= 0:
        return None
    n = geo.ngon_segments(Rmax / MM)
    return revolve_loop(name, [(h0, 0.0), (h0, Rmax), (h1, Rmax), (h1, 0.0)], d, centre, n)


def build_stacked_lathe(name, src_me, d, centre, hex_amp=0.012, bin_mm=0.3, min_len_mm=1.0):
    """Shafts with hex and round lengths (swerve axles, spline shafts): exact hexagonal prisms for the hex lengths,
    lathe rings for the round ones. A length is hex where the outer radius shows a 6-fold pattern."""
    co, tri, _ = geo.mesh_arrays(src_me)
    pts, _ = geo.sample_surface(co, tri, 80000)
    pts = np.vstack([pts, co])
    u, w = geo.perp_basis(d)
    rel = pts - centre
    h = rel @ d
    x, y = rel @ u, rel @ w
    r = np.hypot(x, y)
    th = np.arctan2(y, x)
    h0, h1 = h.min(), h.max()
    nb = int(np.clip((h1 - h0) / (bin_mm * MM), 4, 2000))
    b = np.clip(((h - h0) / (h1 - h0) * nb).astype(int), 0, nb - 1)
    nsec = 36
    s_ = ((th + math.pi) / (2 * math.pi) * nsec).astype(int) % nsec
    smax = np.full((nb, nsec), np.nan)
    np.fmax.at(smax, (b, s_), r)
    ang = (np.arange(nsec) + 0.5) / nsec * 2 * math.pi - math.pi
    kind, rout, phase = [], [], []
    for i in range(nb):
        row = smax[i]
        ok = ~np.isnan(row)
        if ok.sum() < nsec * 0.8:
            kind.append(None); rout.append(np.nan); phase.append(0.0)
            continue
        m = row[ok].mean()
        z = (row[ok] * np.exp(-6j * ang[ok])).mean() / m
        kind.append("hex" if abs(z) > hex_amp else "round")
        rout.append(float(np.median(row[ok])))
        phase.append(float(np.angle(z)))
    # fill gaps, then merge runs shorter than min_len into their neighbour
    for i in range(nb):
        if kind[i] is None:
            kind[i] = next((kind[j] for j in list(range(i, nb)) + list(range(i, -1, -1)) if kind[j]), "round")
    runs, start = [], 0
    for i in range(1, nb + 1):
        if i == nb or kind[i] != kind[start]:
            runs.append([kind[start], start, i])
            start = i
    minb = max(1, int(min_len_mm / bin_mm))
    changed = True
    while changed and len(runs) > 1:
        changed = False
        for j, (k_, a_, b_) in enumerate(runs):
            if b_ - a_ < minb:
                nbr = j - 1 if j > 0 else j + 1
                runs[nbr][1], runs[nbr][2] = min(runs[nbr][1], a_), max(runs[nbr][2], b_)
                runs.pop(j)
                changed = True
                break
        merged = [runs[0]]
        for rr in runs[1:]:
            if rr[0] == merged[-1][0]:
                merged[-1][2] = rr[2]
            else:
                merged.append(rr)
        runs = merged
    edges = h0 + (h1 - h0) * np.arange(nb + 1) / nb
    Rout = np.array(rout)
    Rout = np.where(np.isnan(Rout), np.nanmedian(Rout), Rout)
    Rout = _median_filter(Rout, 3)

    def hex_fit(sel):
        hull = _hull2d(np.c_[x[sel], y[sel]])
        if len(hull) < 3:
            return None
        hx = fit_hexagon(hull)
        c6 = hx.mean(0)
        ap = float(np.linalg.norm((hx[0] + hx[1]) / 2 - c6))
        nrm6 = np.array([((hx[i] + hx[(i + 1) % 6]) / 2 - c6) / ap for i in range(6)])
        dev = float(np.abs(((hull - c6) @ nrm6.T).max(1) - ap).max())
        return None if dev > 0.06 * ap else hx  # 12-tooth splines carry a 6-fold pattern too: only a real hexagon counts

    # a round collar merged into a long hex length (classified hex next to the hex end face) rounds the hull, and the
    # whole 216 mm shaft came out a 16-gon (dev 13% > 6%): retry without the bins standing > 8% above the length's median
    # radius; on success the length splits into hex and round pieces. Lengths that pass the fit are untouched.
    split = []
    for k_, a_, b_ in runs:
        if k_ == "hex" and hex_fit((b >= a_) & (b < b_)) is None:
            core = Rout[a_:b_] <= 1.08 * float(np.median(Rout[a_:b_]))
            in_run = (b >= a_) & (b < b_)
            if core.any() and not core.all() and hex_fit(in_run & core[np.clip(b - a_, 0, b_ - a_ - 1)]) is not None:
                i = a_
                for j in range(a_ + 1, b_ + 1):
                    if j == b_ or core[j - a_] != core[i - a_]:
                        split.append(["hex" if core[i - a_] else "round", i, j])
                        i = j
                continue
        split.append([k_, a_, b_])
    runs = split
    bm = bmesh.new()
    for k_, a_, b_ in runs:
        ha, hb = edges[a_], edges[b_]
        hx = hex_fit((b >= a_) & (b < b_)) if k_ == "hex" else None
        if hx is not None:
            # prism in the (u, w) frame around centre
            c2 = np.array([centre @ u, centre @ w])
            m = build_prism("p", hx + c2, [], u, w, d, ha + centre @ d, hb + centre @ d)
        else:  # round length
            Rm = float(Rout[a_:b_].max())
            tol = max(0.15 * MM, 0.03 * Rm)
            outer = geo.douglas_peucker(stair_to_polyline(edges[a_:b_ + 1], Rout[a_:b_]), tol)
            prof = [(ha, 0.0)] + list(map(tuple, outer)) + [(hb, 0.0)]
            clean = []
            for q in prof:
                if not clean or abs(q[0] - clean[-1][0]) > 1e-7 or abs(q[1] - clean[-1][1]) > 1e-7:
                    clean.append(q)
            m = revolve_loop("r", clean, d, centre, geo.ngon_segments(Rm / MM))
            if "seg" in m.attributes:
                m.attributes.remove(m.attributes["seg"])
        bm.from_mesh(m)
        bpy.data.meshes.remove(m)
    out = bpy.data.meshes.new(name)
    bm.to_mesh(out)
    bm.free()
    out.update()
    return out

# ================================================================ hooks (balls, hex/round shafts, CAD colours)

# ---------------------------------------------------------------- spheres (balls)

def sphere_fit(me):
    """(centre, radius, relative spread) of the surface around its centroid; spread ~0 for a ball."""
    co, tri, _ = geo.mesh_arrays(me)
    pts, _ = geo.sample_surface(co, tri, 4000)
    c = 0.5 * (pts.min(0) + pts.max(0))
    r = np.linalg.norm(pts - c, axis=1)
    return c, float(r.mean()), float(r.std() / max(r.mean(), 1e-9))


def build_sphere(name, centre, radius, segs=None, rings=None):
    """Low-poly UV sphere: segments from the owner's radius tiers, rings = segments / 2."""
    segs = segs or geo.ngon_segments(radius / MM)
    rings = rings or max(3, segs // 2)
    verts, faces = [tuple(centre + np.array((0, 0, -radius)))], []
    for i in range(1, rings):
        phi = -math.pi / 2 + math.pi * i / rings
        for k in range(segs):
            th = 2 * math.pi * k / segs
            verts.append(tuple(centre + radius * np.array((math.cos(phi) * math.cos(th), math.cos(phi) * math.sin(th), math.sin(phi)))))
    verts.append(tuple(centre + np.array((0, 0, radius))))
    top = len(verts) - 1
    ring = lambda i, k: 1 + (i - 1) * segs + (k % segs)
    for k in range(segs):
        faces.append((0, ring(1, k + 1), ring(1, k)))
        faces.append((top, ring(rings - 1, k), ring(rings - 1, k + 1)))
    for i in range(1, rings - 1):
        for k in range(segs):
            faces.append((ring(i, k), ring(i, k + 1), ring(i + 1, k + 1), ring(i + 1, k)))
    me = new_mesh_from(name, verts, faces, smooth_faces=set(range(len(faces))))
    fix_normals(me)
    return me


# ---------------------------------------------------------------- shafts with hex and round sections

def build_sectioned(name, me, hex_ratio=0.99, min_len_mm=1.5):
    """Shaft whose outline changes along its axis (hex body + round journals, round shaft + hex end, gear on a hex
    stub). Each section is its own closed prism: an exact hexagon fitted to that section's outline, or an n-gon cylinder
    (owner's side tiers), keeping the bore where the section has one. Returns the joined mesh (no materials) or None."""
    co, tri, _ = geo.mesh_arrays(me)
    pts, _ = geo.sample_surface(co, tri, 40000)
    # the candidate axis with the best revolve score, not PCA's longest: a gear on a short shaft spreads widest across
    # the shaft, and the PCA axis in the gear plane gave a tilted stack of slabs bigger than the part
    best = (9.0, None)
    for d_ in geo.candidate_axes(pts):
        s_, _ = geo.revolve_score(pts, d_)
        if s_ < best[0]:
            best = (s_, d_)
    d = geo.snap_axis(best[1] if best[1] is not None else geo.pca_axes(pts)[1][0])
    u, w = geo.perp_basis(d)
    allp = np.vstack([pts, co])
    pu, pw = allp @ u, allp @ w
    cu, cw = 0.5 * (pu.min() + pu.max()), 0.5 * (pw.min() + pw.max())
    centre = u * cu + w * cw
    edges, R_out, R_in = revolve_profile(co, tri, d, centre)
    nb = len(R_out)
    h = allp @ d
    h0, h1 = edges[0], edges[-1]
    b = np.clip(((h - h0) / max(h1 - h0, 1e-12) * nb).astype(int), 0, nb - 1)
    # hex vs round on ~1 mm bins (0.15 mm bins leave most of the 72 sectors empty and smear the hex into journals)
    nsec = 72
    nbc = int(max(4, min(400, (h1 - h0) / MM)))
    bc = np.clip(((h - h0) / max(h1 - h0, 1e-12) * nbc).astype(int), 0, nbc - 1)
    r = np.hypot(pu - cu, pw - cw)
    th = np.arctan2(pw - cw, pu - cu)
    s = ((th + math.pi) / (2 * math.pi) * nsec).astype(int) % nsec
    key = bc * nsec + s
    order = np.argsort(key)
    uk, start = np.unique(key[order], return_index=True)
    smax = np.full(nbc * nsec, np.nan)
    smax[uk] = np.maximum.reduceat(r[order], start)
    smax = smax.reshape(nbc, nsec)
    okc = (~np.isnan(smax)).sum(1) >= 0.8 * nsec
    if okc.sum() < 2:
        return None
    # 6-fold harmonic of the outline radius: a hexagon has |c6|/mean ~0.027, a circle ~0 (an off-centre circle only
    # adds the 1st harmonic, which is why a min/max ratio is not used)
    ang = (np.arange(nsec) + 0.5) / nsec * 2 * math.pi - math.pi
    with np.errstate(all="ignore"):
        mean = np.nanmean(smax, axis=1)
        f = np.where(np.isnan(smax), mean[:, None], smax)
        c6 = np.abs((f * np.exp(-6j * ang)[None, :]).mean(1)) / mean
    ic = np.arange(nbc)
    c6 = np.interp(ic, ic[okc], c6[okc])
    centres = (np.arange(nb) + 0.5) / nb
    ratio = 1.0 - c6[np.clip((centres * nbc).astype(int), 0, nbc - 1)]
    Rb = np.maximum(R_out, 0.0)
    kind = _median_filter((ratio < hex_ratio).astype(float), 7) > 0.5
    Rb = _median_filter(Rb, 7)
    Rin = _median_filter(R_in, 7)
    bore = Rin > 0.25 * Rb
    # sections: kind / bore change or a radius step
    secs, a = [], 0
    for i in range(1, nb + 1):
        if i == nb or kind[i] != kind[a] or bore[i] != bore[a] or abs(Rb[i] - Rb[a]) > max(0.3 * MM, 0.04 * Rb[a]):
            secs.append([a, i])
            a = i
    merged = []
    for sct in secs:  # slivers join the previous section
        if merged and (edges[sct[1]] - edges[sct[0]]) < min_len_mm * MM:
            merged[-1][1] = sct[1]
        else:
            merged.append(sct)
    if merged and len(merged) > 1 and (edges[merged[0][1]] - edges[merged[0][0]]) < min_len_mm * MM:
        merged[1][0] = merged[0][0]
        merged.pop(0)
    parts = []
    for a, z in merged:
        ha, hz = edges[a], edges[z]
        # internal junctions overlap (JUNCTION_OVERLAP): no section cap in its neighbour's end-face plane
        ha -= JUNCTION_OVERLAP if a != merged[0][0] else 0.0
        hz += JUNCTION_OVERLAP if z != merged[-1][1] else 0.0
        R = float(np.median(Rb[a:z]))
        sel = (h >= ha) & (h <= hz)
        if kind[(a + z) // 2] and sel.sum() > 10:
            outer = fit_hexagon(_hull2d(np.c_[pu[sel], pw[sel]]))
        else:
            n = geo.ngon_segments(round(R / MM, 2))
            outer = np.array([(cu + R * math.cos(2 * math.pi * k / n), cw + R * math.sin(2 * math.pi * k / n)) for k in range(n)])
        holes = []
        ri = float(np.median(Rin[a:z]))
        if bore[(a + z) // 2] and ri < 0.9 * R:
            n = geo.ngon_segments(ri / MM)
            holes.append(np.array([(cu + ri * math.cos(2 * math.pi * k / n), cw + ri * math.sin(2 * math.pi * k / n)) for k in range(n)]))
        parts.append(build_prism(name, outer, holes, u, w, d, ha, hz))
    import libcombo
    out = libcombo.join_meshes(name, [(m, Matrix.Identity(4)) for m in parts])
    for m in parts:
        bpy.data.meshes.remove(m)
    build_sectioned.sections = [("hex" if kind[(a + z) // 2] else "round", round((edges[z] - edges[a]) / MM, 1)) for a, z in merged]
    return out


# ---------------------------------------------------------------- library parts in the CAD's colours

def recolour_from_cad(src_me, dst_me):
    """Library mesh already placed in the CAD part's frame: each of its material slots takes the CAD material covering
    most of that slot's faces (area-weighted nearest CAD face). The asset keeps its own colour regions (tread vs core),
    with the CAD's colours. Returns {library material: CAD material}."""
    co, tri, mat = geo.mesh_arrays(src_me)
    bvh = BVHTree.FromPolygons([Vector(c) for c in co], [tuple(t) for t in tri])
    votes = {}
    for p in dst_me.polygons:
        loc, nrm, idx, dist = bvh.find_nearest(p.center)
        if idx is None:
            continue
        v = votes.setdefault(p.material_index, {})
        v[int(mat[idx])] = v.get(int(mat[idx]), 0.0) + p.area
    out = {}
    for slot in range(len(dst_me.materials)):
        v = votes.get(slot)
        if v and src_me.materials[max(v, key=v.get)] is not None:
            old = dst_me.materials[slot]
            dst_me.materials[slot] = src_me.materials[max(v, key=v.get)]
            out[old.name if old else str(slot)] = dst_me.materials[slot].name
    return out
