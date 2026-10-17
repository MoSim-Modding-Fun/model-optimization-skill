# Helpers for procedural electronics assets in the library style (LP primitives + Cutters -> one Asset object, RobotMaterials
# finishes linked relative, HP = the source CAD, excluded). Units: build in mm, the asset object is scaled to metres at the end.
# outline(): outer outline of a set of pieces in a height band by rasterising their triangles (0.05 mm), closing the mask
#   (touching pieces merge: a connector filling a housing's opening), tracing the outer contour, smoothing the pixel
#   staircase, Douglas-Peucker. Works on hollow shells, where section loops chain wrongly at T-junctions. Side walls
#   project to thin rings: the area printed is the ring's, the traced outline is still the outer boundary.
# boolean(): EXACT solver with material_mode TRANSFER (the default INDEX maps the operand's faces onto the target's slot
#   0: unions came out in the case colour); view layer + depsgraph updated first (a union on a just-created object was
#   evaluated without its modifier). tri_arrays_mm(keep=...) filters triangles by material name / normal / centroid.
import bpy, bmesh, math, os, json
import numpy as np
from mathutils import Vector, Matrix

LIB_ROOT = None
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"   # "Components", as the other electronics assets


def coll(name, exclude=False):
    c = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    if c.name not in bpy.context.scene.collection.children:
        bpy.context.scene.collection.children.link(c)
    if exclude:
        bpy.context.view_layer.layer_collection.children[c.name].exclude = True
    return c


# ---------------------------------------------------------------- 2D outlines from the CAD
def outer_loop(obj, axis, t_mm, piece_scale=1000.0):
    """Outer loop (largest area) of one piece's section at axis = t_mm, in mm, CCW, as (N, 2) in the other two axes."""
    bm = bmesh.new(); bm.from_mesh(obj.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
    n = Vector((0, 0, 0)); n["xyz".index(axis)] = 1
    res = bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], dist=1e-8,
                                 plane_co=n * (t_mm / piece_scale), plane_no=n)
    edges = [e for e in res["geom_cut"] if isinstance(e, bmesh.types.BMEdge)]
    adj = {}
    for e in edges:
        a, b = e.verts
        adj.setdefault(a, []).append((e, b)); adj.setdefault(b, []).append((e, a))
    used, loops = set(), []
    for e in edges:
        if e in used:
            continue
        used.add(e); a, b = e.verts; lp = [a, b]; cur = b
        while True:
            nx = [(f, w) for f, w in adj.get(cur, []) if f not in used]
            if not nx:
                break
            f, w = nx[0]; used.add(f)
            if w is lp[0]:
                break
            lp.append(w); cur = w
        if len(lp) > 2:
            loops.append(lp)
    u, v = [k for k in range(3) if k != "xyz".index(axis)]
    best = None
    for lp in loops:
        P = np.array([[x.co[u], x.co[v]] for x in lp]) * piece_scale
        A = area(P)
        if best is None or abs(A) > abs(best[0]):
            best = (A, P)
    bm.free()
    P = best[1]
    return P if best[0] > 0 else P[::-1]


def area(P):
    x, y = P[:, 0], P[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def dp(P, tol):
    """Douglas-Peucker on a closed loop (start at the point farthest from the centroid)."""
    k = int(np.argmax(np.linalg.norm(P - P.mean(0), axis=1)))
    P = np.roll(P, -k, axis=0)
    Q = np.vstack([P, P[:1]])

    def rec(a, b):
        if b <= a + 1:
            return [a]
        seg = Q[b] - Q[a]; L = np.linalg.norm(seg)
        D = Q[a + 1:b] - Q[a]
        d = np.abs(seg[0] * D[:, 1] - seg[1] * D[:, 0]) / L if L > 1e-12 else np.linalg.norm(D, axis=1)
        i = int(np.argmax(d))
        if d[i] > tol:
            m = a + 1 + i
            return rec(a, m) + rec(m, b)
        return [a]
    idx = rec(0, len(Q) - 1)
    return P[idx]


def circle(c, r, n, start=0.0):
    a = start + np.arange(n) * 2 * math.pi / n
    return np.stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)], 1)


def obround(c, half_len, r, ang, n_end=6):
    """Slot outline: centre, half distance between the end centres, end radius, direction angle (rad)."""
    d = np.array([math.cos(ang), math.sin(ang)]); nrm = np.array([-d[1], d[0]])
    pts = []
    for s, base in ((1, ang - math.pi / 2), (-1, ang + math.pi / 2)):
        cc = np.array(c) + s * half_len * d
        for k in range(n_end + 1):
            a = base + k * math.pi / n_end
            pts.append(cc + r * np.array([math.cos(a), math.sin(a)]))
    return np.array(pts)


# ---------------------------------------------------------------- solids
def prism(name, P, z0, z1, mat_index=0, collection=None, axis="z"):
    """Closed prism from a CCW 2D loop P (mm) between z0 and z1 along `axis` (outline in the other two axes)."""
    bm = bmesh.new()
    n = len(P)
    lo = [bm.verts.new(_v3(p, z0, axis)) for p in P]
    hi = [bm.verts.new(_v3(p, z1, axis)) for p in P]
    f0 = bm.faces.new(lo[::-1]); f1 = bm.faces.new(hi)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((lo[i], lo[j], hi[j], hi[i]))
    for f in bm.faces:
        f.material_index = mat_index
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
    ob = bpy.data.objects.new(name, me)
    (collection or bpy.context.scene.collection).objects.link(ob)
    return ob


def _v3(p, z, axis):
    if axis == "z":
        return (p[0], p[1], z)
    if axis == "y":
        return (p[0], z, p[1])
    return (z, p[0], p[1])


def box(name, lo, hi, mat_index=0, collection=None, bevel=0.0, bevel_segments=1):
    P = np.array([[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]])
    ob = prism(name, P, lo[2], hi[2], mat_index, collection)
    if bevel > 0:
        bm = bmesh.new(); bm.from_mesh(ob.data)
        bmesh.ops.bevel(bm, geom=bm.edges[:], offset=bevel, segments=bevel_segments, affect='EDGES', profile=0.5)
        bm.to_mesh(ob.data); bm.free()
    return ob


def boolean(target, cutter, op='DIFFERENCE'):
    m = target.modifiers.new("bool", 'BOOLEAN')
    m.operation = op; m.solver = 'EXACT'; m.object = cutter; m.material_mode = 'TRANSFER'
    try:
        m.use_self = False; m.use_hole_tolerant = False
    except AttributeError:
        pass
    bpy.context.view_layer.objects.active = target
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    dg.update()
    me = bpy.data.meshes.new_from_object(target.evaluated_get(dg), depsgraph=dg)
    n0, n1 = len(target.data.polygons), len(me.polygons)
    if n0 == n1 and op == 'UNION':
        print("BOOLEAN warning: union with", cutter.name, "left the face count unchanged", flush=True)
    target.modifiers.remove(m)
    old = target.data; target.data = me; bpy.data.meshes.remove(old)
    return target


def set_materials(ob, mats):
    ob.data.materials.clear()
    for m in mats:
        ob.data.materials.append(m)


def join(name, objs, collection):
    """Copy of the objects' meshes joined into one object (inputs untouched, so LP/Cutters stay as they are)."""
    bm = bmesh.new()
    mats = []
    for o in objs:
        me = o.data.copy(); me.transform(o.matrix_world)
        idx_map = []
        for m in me.materials:
            if m not in mats:
                mats.append(m)
            idx_map.append(mats.index(m))
        tmp = bmesh.new(); tmp.from_mesh(me)
        for f in tmp.faces:
            f.material_index = idx_map[f.material_index] if idx_map else 0
        tmp.to_mesh(me); tmp.free()
        bm.from_mesh(me)
        bpy.data.meshes.remove(me)
    out = bpy.data.meshes.new(name); bm.to_mesh(out); bm.free()
    for m in mats:
        out.materials.append(m)
    ob = bpy.data.objects.new(name, out); collection.objects.link(ob)
    return ob


def tidy(ob, angle_deg=1.0):
    bm = bmesh.new(); bm.from_mesh(ob.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
    bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(angle_deg), use_dissolve_boundaries=False,
                             verts=bm.verts, edges=bm.edges, delimit={'MATERIAL'})
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    # concave n-gons triangulated (beauty) so every importer agrees on them
    concave = [f for f in bm.faces if len(f.verts) > 4 and not _convex(f)]
    if concave:
        bmesh.ops.triangulate(bm, faces=concave, quad_method='BEAUTY', ngon_method='BEAUTY')
    bm.to_mesh(ob.data); bm.free()
    ob.data.update()


def _convex(f):
    n = f.normal
    vs = [v.co for v in f.verts]
    for i in range(len(vs)):
        a, b, c = vs[i - 1], vs[i], vs[(i + 1) % len(vs)]
        if (b - a).cross(c - b).dot(n) < -1e-9:
            return False
    return True


def tris(ob):
    return sum(len(p.vertices) - 2 for p in ob.data.polygons)


def open_edges(ob):
    bm = bmesh.new(); bm.from_mesh(ob.data)
    n = sum(1 for e in bm.edges if len(e.link_faces) != 2)
    bm.free()
    return n


def robot_materials(names, lib_blend, relative=True):
    with bpy.data.libraries.load(lib_blend, link=True, relative=relative) as (src, dst):
        dst.materials = [n for n in names if n in src.materials]
    return {m.name: m for m in dst.materials}


def finish_asset(ob, name, description, tags):
    ob.name = name; ob.data.name = name
    ob.asset_mark()
    ad = ob.asset_data
    ad.catalog_id = CATALOG
    ad.description = description
    for t in tags:
        ad.tags.new(t, skip_if_exists=True)


# ---------------------------------------------------------------- silhouettes (outer outlines of hollow shells)
def tri_arrays_mm(objs, keep=None):
    """World vertices (mm) and triangles of the objects; keep(material name, normals (n,3), centroids (n,3)) -> bool mask
    filters triangles (e.g. one finish facing one way)."""
    V, T = [], []
    for o in objs:
        m = o.data; m.calc_loop_triangles()
        co = np.array([tuple(o.matrix_world @ v.co) for v in m.vertices]) * 1000
        tri = np.array([tuple(t.vertices) for t in m.loop_triangles])
        if keep is not None:
            mi = np.array([t.material_index for t in m.loop_triangles])
            A, B, C = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
            cr = np.cross(B - A, C - A); nn = cr / np.maximum(np.linalg.norm(cr, axis=1)[:, None], 1e-12)
            cen = (A + B + C) / 3
            sel = np.zeros(len(tri), bool)
            for k, mat in enumerate(m.materials):
                mk = mi == k
                if mk.any():
                    sel[mk] = keep(mat.name if mat else "-", nn[mk], cen[mk])
            tri = tri[sel]
        T.append(tri + sum(len(v) for v in V))
        V.append(co)
    return np.vstack(V), np.vstack(T)


def silhouette_mask(objs, axis="z", band=None, res=0.05, pad=1.0, V=None, T=None):
    """Binary mask of the triangles' projection along `axis` (only triangles touching the band, clipped by centroid of
    the band test), grid origin and resolution returned. Holes are kept (flood fill decides the outside)."""
    if V is None:
        V, T = tri_arrays_mm(objs)
    k = "xyz".index(axis); u, v = [i for i in range(3) if i != k]
    A, B, C = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    if band is not None:
        lo = np.minimum(np.minimum(A[:, k], B[:, k]), C[:, k]); hi = np.maximum(np.maximum(A[:, k], B[:, k]), C[:, k])
        sel = (hi >= band[0]) & (lo <= band[1])
        A, B, C = A[sel], B[sel], C[sel]
    P = np.vstack([A, B, C])
    o = np.array([P[:, u].min() - pad, P[:, v].min() - pad])
    nx = int((P[:, u].max() + pad - o[0]) / res) + 1; ny = int((P[:, v].max() + pad - o[1]) / res) + 1
    mask = np.zeros((ny, nx), bool)
    for a, b, c in zip(A[:, [u, v]], B[:, [u, v]], C[:, [u, v]]):
        mn = np.floor((np.minimum(np.minimum(a, b), c) - o) / res).astype(int)
        mx = np.ceil((np.maximum(np.maximum(a, b), c) - o) / res).astype(int)
        xs = np.arange(mn[0], mx[0] + 1); ys = np.arange(mn[1], mx[1] + 1)
        if len(xs) == 0 or len(ys) == 0:
            continue
        X, Y = np.meshgrid(o[0] + (xs + 0.5) * res, o[1] + (ys + 0.5) * res)
        d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(d) < 1e-12:
            continue
        l1 = ((b[1] - c[1]) * (X - c[0]) + (c[0] - b[0]) * (Y - c[1])) / d
        l2 = ((c[1] - a[1]) * (X - c[0]) + (a[0] - c[0]) * (Y - c[1])) / d
        inside = (l1 >= -1e-6) & (l2 >= -1e-6) & (1 - l1 - l2 >= -1e-6)
        mask[ys[0]:ys[-1] + 1, xs[0]:xs[-1] + 1] |= inside
    return mask, o, res


def components(mask):
    """Label 4-connected components (iterative flood fill). Returns label array and sizes."""
    lab = np.zeros(mask.shape, np.int32); n = 0; sizes = []
    H, W = mask.shape
    for y0, x0 in zip(*np.nonzero(mask)):
        if lab[y0, x0]:
            continue
        n += 1; stack = [(y0, x0)]; lab[y0, x0] = n; cnt = 0
        while stack:
            y, x = stack.pop(); cnt += 1
            for yy, xx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
                if 0 <= yy < H and 0 <= xx < W and mask[yy, xx] and not lab[yy, xx]:
                    lab[yy, xx] = n; stack.append((yy, xx))
        sizes.append(cnt)
    return lab, sizes


def trace(mask, o, res):
    """Outer boundary of a binary blob (Moore-neighbour tracing on pixel corners), as mm coordinates, CCW."""
    H, W = mask.shape
    m = np.zeros((H + 2, W + 2), bool); m[1:-1, 1:-1] = mask
    ys, xs = np.nonzero(m)
    start = (ys[0], xs[0])            # top-most row (smallest y index), left-most pixel
    # walk pixel-edge boundary: directions on the corner grid
    # represent the boundary as edges between a filled pixel and an empty one, chained clockwise in image space
    edges = {}
    for y, x in zip(ys, xs):
        if not m[y - 1, x]: edges[(y, x)] = (y, x + 1)            # top edge, left->right
        if not m[y, x + 1]: edges[(y, x + 1)] = (y + 1, x + 1)    # right edge, top->bottom
        if not m[y + 1, x]: edges[(y + 1, x + 1)] = (y + 1, x)    # bottom edge, right->left
        if not m[y, x - 1]: edges[(y + 1, x)] = (y, x)            # left edge, bottom->top
    # NOTE: corner keys can collide at diagonal pinch points; good enough for blobs with smooth outlines
    p0 = (start[0], start[1]); pts = [p0]; cur = edges.get(p0)
    guard = 0
    while cur is not None and cur != p0 and guard < 10 ** 7:
        pts.append(cur); cur = edges.get(cur); guard += 1
    P = np.array([[o[0] + (x - 1) * res, o[1] + (y - 1) * res] for y, x in pts])
    return P if area(P) > 0 else P[::-1]


def smooth_loop(P, w):
    """Circular moving average over w points (removes the pixel staircase of traced outlines)."""
    if w <= 1 or len(P) < 3 * w:
        return P
    k = np.ones(w) / w
    ext = np.vstack([P[-w:], P, P[:w]])
    return np.stack([np.convolve(ext[:, i], k, mode="same")[w:-w] for i in range(2)], 1)


def close_mask(mask, k):
    """Morphological closing with a diamond of radius k pixels (merges blobs closer than ~2k px, fills thin notches)."""
    if k <= 0:
        return mask
    m = np.pad(mask, k + 1)
    for _ in range(k):
        m = m | np.roll(m, 1, 0) | np.roll(m, -1, 0) | np.roll(m, 1, 1) | np.roll(m, -1, 1)
    for _ in range(k):
        m = m & np.roll(m, 1, 0) & np.roll(m, -1, 0) & np.roll(m, 1, 1) & np.roll(m, -1, 1)
    return m[k + 1:-k - 1, k + 1:-k - 1]


def outline(objs, axis, band, res=0.05, tol=0.2, min_px=200, V=None, T=None, smooth=7, close=10):
    """Outer outlines (one per connected blob of at least min_px pixels) of the objects' projection in a band,
    Douglas-Peucker simplified (tol mm). Returns list of (area mm2, loop) sorted by area."""
    mask, o, r = silhouette_mask(objs, axis, band, res, V=V, T=T)
    mask = close_mask(mask, close)
    lab, sizes = components(mask)
    out = []
    for i, s in enumerate(sizes, 1):
        if s < min_px:
            continue
        L = smooth_loop(trace(lab == i, o, r), smooth)
        Q = dp(L, tol)
        out.append((s * r * r, Q))
    return sorted(out, key=lambda t: -t[0])


def chain(segs, tol=1e-3):
    """Closed loops from section segments [(p, q), ...] (2D, mm) by matching end points (one simple piece only)."""
    from collections import defaultdict
    key = lambda p: (round(p[0] / tol), round(p[1] / tol))
    adj = defaultdict(list)
    for i, (p, q) in enumerate(segs):
        adj[key(p)].append((i, q)); adj[key(q)].append((i, p))
    used, out = set(), []
    for i, (p, q) in enumerate(segs):
        if i in used:
            continue
        used.add(i); lp = [p, q]; cur = q
        while True:
            nx = [(j, r) for j, r in adj[key(cur)] if j not in used]
            if not nx:
                break
            j, r = nx[0]; used.add(j)
            if key(r) == key(lp[0]):
                break
            lp.append(r); cur = r
        if len(lp) > 2:
            out.append(np.array(lp))
    return out
