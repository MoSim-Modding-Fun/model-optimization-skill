# Clean "Kraken-style" remodels of library assets (owner 2026-10-10: "Remodel these to be like the kraken and
# similar. you can omit the port."): a few plain shapes instead of the per-piece simplified CAD, the asset object, its
# modifiers (Smooth by Angle + WeightedNormal), material slots and asset data kept, only its mesh replaced.
#   blender -b "<library asset>.blend" --python clean_remodel.py -- motor|battery <work.blend> <out stage.blend> [--write]
# motor:   one lathe (end cap, can, label band, gearbox barrel: build_revolve remodel) + the gearbox face (its outline
#          and front bosses stacked along the axis, holes filled) + the output shaft as a hex prism. Screws, pins,
#          bearings and the encoder port (Plastic (1)) are left out. Axis = +y of the asset frame (kit_extract key piece).
# battery: one loft of the shrink-wrap's horizontal sections (the cell waves kept, the wire channel at the end filled),
#          rounded top / bottom edges from the CAD sections. The label strip and the leads (the port) are left out.
# CAD piece-name prefixes come from CLEAN_PIECES (JSON). motor: {"lathe": [prefixes of the body bands],
# "barrel", "face", "shaft", "endcap": one prefix each}; battery: {"wrap": shrink-wrap piece, "cells": cell block}.
# Materials: each new face takes the finish of the nearest face of the current library mesh (same frame).
# Without --write only <out stage.blend> is saved (new mesh "<asset> [clean]" + the old asset) for review.
import bpy, bmesh, sys, os, math, json
import numpy as np
from mathutils import Vector, Matrix
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geo, build, remodel as R
MM = 1e-3
a = sys.argv[sys.argv.index("--") + 1:]
KIND, WORK, STAGE = a[0], a[1], a[2]
WRITE = "--write" in a
if "CLEAN_PIECES" not in os.environ:
    raise SystemExit("set CLEAN_PIECES to the JSON map of CAD piece-name prefixes (see the header)")
PC = json.loads(os.environ["CLEAN_PIECES"])
asset = next(o for o in bpy.data.objects if o.asset_data and o.type == 'MESH')
with bpy.data.libraries.load(WORK, link=False) as (src, dst):
    dst.objects = list(src.objects)
_tmp = bpy.data.collections.new("__cad_tmp"); bpy.context.scene.collection.children.link(_tmp)
for o in dst.objects:
    if o:
        _tmp.objects.link(o)
bpy.context.view_layer.update()  # parent chains -> matrix_world (appended objects outside a scene are not evaluated)
cad = [o for o in dst.objects if o and o.type == 'MESH']


def baked(objs):
    """One mesh of these CAD pieces in the asset frame (work.blend world = asset frame), doubles welded."""
    bm = bmesh.new()
    for o in objs:
        me = o.data.copy(); me.transform(o.matrix_world)
        if "custom_normal" in me.attributes:
            me.attributes.remove(me.attributes["custom_normal"])
        bm.from_mesh(me); bpy.data.meshes.remove(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)
    out = bpy.data.meshes.new("baked"); bm.to_mesh(out); bm.free()
    mats = []
    for o in objs:
        for s in o.material_slots:
            if s.material and s.material not in mats:
                mats.append(s.material)
    return out


def pieces(*prefixes, exclude=()):
    return [o for o in cad if o.name.startswith(prefixes) and not o.name.startswith(exclude)]


def tris(me):
    return sum(len(p.vertices) - 2 for p in me.polygons)


def join(meshes, name):
    bm = bmesh.new()
    for m in meshes:
        bm.from_mesh(m)
    out = bpy.data.meshes.new(name); bm.to_mesh(out); bm.free()
    return out


def hex_prism(name, sec_loop, y0, y1):
    """Hex prism along +y from the shaft's section loop (corners at its max-radius directions)."""
    P = np.asarray(sec_loop)  # (u, w) = section basis of +y: u = -z, w = -x
    r = np.linalg.norm(P, axis=1)
    ang = math.atan2(P[np.argmax(r)][1], P[np.argmax(r)][0])
    rr = float(r.max())
    u, w = geo.perp_basis(np.array([0, 1.0, 0]))
    ring = [u * rr * math.cos(ang + k * math.pi / 3) + w * rr * math.sin(ang + k * math.pi / 3) for k in range(6)]
    bm = bmesh.new()
    lo = [bm.verts.new(Vector(p) + Vector((0, y0, 0))) for p in ring]
    hi = [bm.verts.new(Vector(p) + Vector((0, y1, 0))) for p in ring]
    for k in range(6):
        bm.faces.new((lo[k], lo[(k + 1) % 6], hi[(k + 1) % 6], hi[k]))
    bm.faces.new(hi)  # tip cap; the root end sits inside the gearbox face
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    out = bpy.data.meshes.new(name); bm.to_mesh(out); bm.free()
    return out, rr


def port_inset(cap_me, y_back, rmax):
    """Cutter for the encoder-port pocket in the end cap's back (owner: "this can be inset"): the cap's sections near the
    back face are a disc with a notch; the notch (outline points 0.4 mm inside the cap radius) pushed out past the rim is
    extruded from 1 mm behind the back face to where the sections are a full disc again. None if there is no notch."""
    d = np.array([0, 1.0, 0])
    def notch(y):
        loops, u, w = build.section_loops(cap_me, d, y)
        if not loops:
            return None, u, w
        Lp = np.asarray(max(loops, key=lambda l: abs(geo.poly_area(np.asarray(l)))), float)
        r = np.linalg.norm(Lp, axis=1)
        return (Lp, r) if (r < rmax - 0.4 * MM).any() else None, u, w
    ys = [y_back + k * 0.5 * MM for k in range(1, 40)]
    shape, floor = None, None
    for y in ys:
        n_, u, w = notch(y)
        if n_ is None:
            floor = y; break
        if shape is None and y >= y_back + 2.5 * MM:
            shape = n_
    if shape is None or floor is None:
        return None, None
    Lp, r = shape
    if geo.poly_area(Lp) < 0:
        Lp, r = Lp[::-1], r[::-1]
    inside = r < rmax - 0.4 * MM
    # the notch as one chain: rotate the loop to start just after the longest run of rim points
    k = int(np.argmax(~inside)); n = len(Lp)
    order = [(k + i) % n for i in range(n)]
    while inside[order[0]]:
        order = order[1:] + order[:1]
    idx = [i for i in order if inside[i]]
    first, last = order.index(idx[0]), order.index(idx[-1])
    chain = Lp[[order[i] for i in range(max(first - 1, 0), min(last + 2, n))]]
    chain = geo.douglas_peucker(chain, 0.4 * MM)
    a0, a1 = math.atan2(chain[0][1], chain[0][0]), math.atan2(chain[-1][1], chain[-1][0])
    if a1 < a0:
        a1 += 2 * math.pi
    R = rmax + 3 * MM
    arc = [(R * math.cos(t), R * math.sin(t)) for t in np.linspace(a1, a0, 6)]
    poly = [tuple(p) for p in chain] + arc  # notch boundary, then back around outside the rim
    if geo.poly_area(np.array(poly)) < 0:
        poly = poly[::-1]
    bm = bmesh.new()
    lo = [bm.verts.new(Vector(u * p[0] + w * p[1] + d * (y_back - 1 * MM))) for p in poly]
    hi = [bm.verts.new(Vector(u * p[0] + w * p[1] + d * floor)) for p in poly]
    m = len(poly)
    for i in range(m):
        bm.faces.new((lo[i], lo[(i + 1) % m], hi[(i + 1) % m], hi[i]))
    bm.faces.new(lo[::-1]); bm.faces.new(hi)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    out = bpy.data.meshes.new("port cutter"); bm.to_mesh(out); bm.free()
    return out, {"floor_mm": round(floor / MM, 2), "chain_pts": len(chain), "depth_mm": round((floor - y_back) / MM, 2)}


def clip_y(me, y, keep_above=True):
    bm = bmesh.new(); bm.from_mesh(me)
    r = bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], plane_co=(0, y, 0), plane_no=(0, 1, 0),
                               clear_inner=keep_above, clear_outer=not keep_above)
    edges = [e for e in r["geom_cut"] if isinstance(e, bmesh.types.BMEdge)]
    if edges:
        bmesh.ops.holes_fill(bm, edges=edges)
    out = bpy.data.meshes.new(me.name + " clip"); bm.to_mesh(out); bm.free()
    return out


info = {"kind": KIND, "old_tris": tris(asset.data)}
if KIND == "motor":
    lathe_src = baked(pieces(*PC["lathe"]))
    for o in pieces(*PC["lathe"]):
        for s in o.material_slots:
            if s.material and s.material.name not in [m.name for m in lathe_src.materials]:
                lathe_src.materials.append(s.material)
    barrel = pieces(PC["barrel"])[0]
    face = pieces(PC["face"])[0]
    shaft = pieces(PC["shaft"])[0]
    yb = max((barrel.matrix_world @ v.co).y for v in barrel.data.vertices)
    fme = baked([face]); yf = max(v.co.y for v in fme.vertices)
    sme = baked([shaft]); ys = max(v.co.y for v in sme.vertices)
    # remodel=False: the groove closing filled the bare can between end cap, label band and barrel (0.7-1 mm steps)
    L = build.build_revolve("lathe", lathe_src, np.array([0, 1.0, 0]), np.zeros(3), False, toothed=False, remodel=False,
                            tol_mm=float(os.environ.get("LATHE_TOL", 0.25)))
    # the encoder port: not modelled, its pocket in the end cap's back cut as a plain inset (owner: "this can be inset")
    capme = baked(pieces(PC.get("endcap", "endcap")))
    yc = min(v.co.y for v in capme.vertices); rc = max(math.hypot(v.co.x, v.co.z) for v in capme.vertices)
    cut, pinfo = port_inset(capme, yc, rc)
    if cut is not None and not os.environ.get("NO_PORT_INSET"):
        L2 = R.boolean(L, cut, "lathe inset")
        if build.open_edges(L2) == 0:
            bpy.data.meshes.remove(L); L = L2
        info_port = pinfo
    else:
        info_port = None
    front = clip_y(fme, yb - 0.05 * MM)  # the part of the face outside the barrel
    if os.environ.get("CLEAN_DEBUG"):
        _c = np.array([v.co for v in fme.vertices]); print("DBG face", _c.min(0) / MM, _c.max(0) / MM, "yb", yb / MM, "front verts", len(front.vertices))
    F, finfo = R.remodel("face", front, "y", tol_mm=float(os.environ.get("FACE_TOL", 0.8)), cross=False, drop_holes_mm2=400.0,
                         merge_mm=float(os.environ.get("FACE_MERGE", 2.0)), snap=True)
    loops, u_, w_ = build.section_loops(sme, np.array([0, 1.0, 0]), (yf + ys) / 2)
    Hx, rr = hex_prism("shaft", max(loops, key=lambda l: abs(geo.poly_area(np.asarray(l)))), yf - 0.5 * MM, ys)
    new = join([L, F, Hx], asset.name + " [clean]")
    info.update(port=info_port, lathe=tris(L), face=tris(F), face_info={k: finfo[k] for k in ("levels", "runs", "err_mm")}, shaft=tris(Hx),
                shaft_r_mm=round(rr / MM, 2), barrel_end_mm=round(yb / MM, 2), face_front_mm=round(yf / MM, 2))
elif KIND == "battery":
    wrap = pieces(PC["wrap"])[0]
    me = baked([wrap])
    cells = baked(pieces(PC["cells"]))
    co = np.array([v.co for v in me.vertices]); z0, z1 = float(co[:, 2].min()), float(co[:, 2].max())
    d = np.array([0, 0, 1.0])
    N = int(os.environ.get("BATT_N", 64))  # 96 left a step at the wire-channel end; 64 is cleaner and lighter

    def outline(z):
        """The pack's outline at height z: N points, CCW, arc-length spaced from the +u ray of the centroid. The wrap is
        a thin shell open at the ends (its sections are strips, not outlines): the outline is the angular envelope of
        all section points of the wrap and the cell block, M directions around the pack's centre."""
        pts = []
        for m_ in (me, cells):
            loops, u, w = build.section_loops(m_, d, z)
            for l in loops:
                l = np.asarray(l, float)
                # densify so long straight segments still reach every direction bin
                for p0, p1 in zip(l, np.roll(l, -1, 0)):
                    k = max(1, int(np.linalg.norm(p1 - p0) / (0.5 * MM)))
                    pts += [p0 + (p1 - p0) * (i / k) for i in range(k)]
        pts = np.array(pts)
        c2 = (pts.max(0) + pts.min(0)) / 2
        M = 360
        th = np.arctan2(pts[:, 1] - c2[1], pts[:, 0] - c2[0]); rr = np.linalg.norm(pts - c2, axis=1)
        b_ = ((th + math.pi) / (2 * math.pi) * M).astype(int) % M
        rmax = np.zeros(M)
        np.maximum.at(rmax, b_, rr)
        for i in np.where(rmax == 0)[0]:
            rmax[i] = max(rmax[i - 1], rmax[(i + 1) % M])
        rmax = np.maximum.reduce([np.roll(rmax, k) for k in (-1, 0, 1)])  # narrow notches filled
        ang = -math.pi + (np.arange(M) + 0.5) / M * 2 * math.pi
        Lp = np.c_[c2[0] + rmax * np.cos(ang), c2[1] + rmax * np.sin(ang)]
        # fill the wire channel at the end (the port): outline points inside the convex hull go to it, more than 0.4 mm
        # in the end zones (outer 15% along the long axis), more than 2.5 mm elsewhere (the cell waves stay)
        from mathutils.geometry import convex_hull_2d
        cc = Lp.mean(0); ev = np.linalg.eigh(np.cov((Lp - cc).T))[1][:, -1]; half = np.abs((Lp - cc) @ ev).max()
        thr = lambda p: (0.4 if abs((p - cc) @ ev) > 0.85 * half else 2.5) * MM
        H = Lp[convex_hull_2d([tuple(p) for p in Lp])]
        Hc = np.vstack([H, H[:1]])
        out = []
        for p in Lp:
            best = None
            for a0, b0 in zip(Hc[:-1], Hc[1:]):
                ab = b0 - a0; t = np.clip((p - a0) @ ab / max(ab @ ab, 1e-18), 0, 1); q = a0 + ab * t
                dd = np.linalg.norm(q - p)
                if best is None or dd < best[0]:
                    best = (dd, q)
            out.append(best[1] if best[0] > thr(p) else p)
        Lp = np.array(out)
        # start where the ray from the centroid along +u leaves the outline (consistent from ring to ring)
        seg = np.linalg.norm(np.roll(Lp, -1, 0) - Lp, axis=1); cum = np.r_[0, np.cumsum(seg)]
        cen = Lp.mean(0); best = None
        for j in range(len(Lp)):
            p0, p1 = Lp[j], Lp[(j + 1) % len(Lp)]
            if (p0[1] - cen[1]) * (p1[1] - cen[1]) <= 0 and abs(p1[1] - p0[1]) > 1e-12:
                t = (cen[1] - p0[1]) / (p1[1] - p0[1]); x = p0[0] + (p1[0] - p0[0]) * t
                if x > cen[0] and (best is None or x > best[0]):
                    best = (x, j, t)
        s0 = cum[best[1]] + best[2] * seg[best[1]]; tot = cum[-1]
        P = []
        for i in range(N):
            s_ = (s0 + tot * i / N) % tot
            j = min(int(np.searchsorted(cum, s_, side='right') - 1), len(Lp) - 1)
            t = (s_ - cum[j]) / max(seg[j], 1e-12)
            P.append(Lp[j] + (Lp[(j + 1) % len(Lp)] - Lp[j]) * t)
        return np.array(P), u, w
    # one outline (mid height, where the wrap is a clean closed loop: the sections near the rounded bottom split it),
    # inset for the rounded top / bottom edges (fillet RF): every ring has the same vertices, so the loft cannot twist
    base, u, w = outline((z0 + z1) / 2)
    nrm = []
    for i in range(N):
        t = base[(i + 1) % N] - base[i - 1]; n_ = np.array([t[1], -t[0]]); nrm.append(n_ / max(np.linalg.norm(n_), 1e-12))
    nrm = np.array(nrm)  # outward for a CCW outline
    RF = float(os.environ.get("BATT_FILLET_MM", 3.5)) * MM

    def ring(z):
        h = min(z - z0, z1 - z)
        inset = RF - math.sqrt(max(RF * RF - (RF - h) ** 2, 0.0)) if h < RF else 0.0
        P = base - nrm * inset
        return [Vector(u * p[0] + w * p[1] + d * z) for p in P]
    # heights: the fillet as 3 steps at each end (angles 30 / 60 / 90 deg of the arc)
    zs = sorted({z0 + RF * (1 - math.cos(math.radians(a_))) for a_ in (0, 45, 90)} | {z1 - RF * (1 - math.cos(math.radians(a_))) for a_ in (0, 45, 90)})
    zs = [min(max(z, z0 + 0.05 * MM), z1 - 0.05 * MM) for z in zs]
    bm = bmesh.new()
    rings = [[bm.verts.new(p) for p in ring(z)] for z in zs]
    for r0, r1 in zip(rings[:-1], rings[1:]):
        for i in range(N):
            bm.faces.new((r0[i], r0[(i + 1) % N], r1[(i + 1) % N], r1[i]))
    bm.faces.new(rings[0][::-1]); bm.faces.new(rings[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    # straight runs of the outline do not need every resampled vertex: dissolve near-flat edges (keeps the waves)
    bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(2.0), verts=bm.verts, edges=bm.edges)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    new = bpy.data.meshes.new(asset.name + " [clean]"); bm.to_mesh(new); bm.free()
    info.update(rings=len(zs), N=N)
else:
    raise SystemExit("kind: motor | battery")
# shading + materials as the library asset: smooth faces, the asset's own Smooth by Angle / WeightedNormal stack
for p in new.polygons:
    p.use_smooth = True
new.set_sharp_from_angle(angle=math.radians(30))
build.transfer_materials(asset.data, new)
info["new_tris"] = tris(new); info["open_edges"] = build.open_edges(new)
e1, e2 = build.fit_error(asset.data, new)
info["vs_old_mm"] = [round(e1, 2), round(e2, 2)]
print("CLEAN", json.dumps(info), flush=True)
if WRITE:
    old = asset.data
    asset.data = new
    new.name = old.name
    bpy.data.meshes.remove(old)
    for o in dst.objects:
        if o and o.name in bpy.data.objects:
            bpy.data.objects.remove(o)
    bpy.data.collections.remove(_tmp)
    bpy.ops.outliner.orphans_purge(do_recursive=True)
    bpy.ops.wm.save_mainfile()
    print("CLEAN wrote", bpy.data.filepath, flush=True)
else:
    o = bpy.data.objects.new(new.name, new); bpy.context.scene.collection.objects.link(o)
    o.matrix_world = asset.matrix_world
    for x in dst.objects:
        if x and x.name in bpy.data.objects:
            bpy.data.objects.remove(x)
    bpy.data.collections.remove(_tmp)
    bpy.ops.wm.save_as_mainfile(filepath=STAGE, copy=True)
    print("CLEAN stage", STAGE, flush=True)
