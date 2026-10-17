# Full simplification pipeline. Run:  blender -b <src.blend> --python pipeline.py -- vis.json out.blend report.json
import bpy, bmesh, sys, os, json, math, time, re, collections
import numpy as np
from mathutils import Vector, Matrix

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geo, build, align, robots, libcombo
from mathutils.bvhtree import BVHTree
cfg = robots.load()
build.TUBE_MITRED = getattr(cfg, "TUBE_MITRED", False)  # mitred tubes -> wall slabs (build.analyse_tube)
build.HEX_BODY_FIT = getattr(cfg, "HEX_BODY_FIT", False)  # hex fitted to the shaft body, not the end clips

VIS, OUT, REPORT = sys.argv[sys.argv.index("--") + 1:][:3]
t_start = time.time()
sc = bpy.context.scene
dg = bpy.context.evaluated_depsgraph_get()
MM = 0.001


def log(*a):
    print(f"[{time.time() - t_start:6.1f}s]", *a, flush=True)


def part(o):
    return cfg.part(o.name)


def eval_tris(o):
    ev = o.evaluated_get(dg)
    m = ev.to_mesh()
    n = sum(len(p.vertices) - 2 for p in m.polygons)
    ev.to_mesh_clear()
    return n


objs = [o for o in sc.objects if o.type == 'MESH' and len(o.data.polygons) > 0]  # exploded parents (explode.py) are empty
CAD_BOX = {}  # world bbox of every CAD object, taken before any mesh is swapped
# multi-piece library swaps (MAXPlanetary, Minion): the pieces are planned like any part; afterwards the library
# mesh goes onto one piece (or a new object) and the other pieces move to "REVIEW - Replaced by library"
ASM = []
for spec in cfg.LIB_ASSEMBLIES:
    pieces = [o for o in objs if spec["select"](o)]
    groups = spec["groups"](pieces) if "groups" in spec else ([pieces] if pieces else [])
    ASM += [(spec, g) for g in groups if g]
users = collections.defaultdict(list)
for o in objs:
    users[o.data.name].append(o)
user_done = {o.name for o in objs if o.modifiers}
orig_tris = {o.name: eval_tris(o) for o in objs}
mats_before = set(bpy.data.materials.keys())
log("objects", len(objs), "meshes", len(users), "tris", sum(orig_tris.values()))

# ------------------------------------------------------------------ review flags
vis = json.load(open(VIS))
full = vis["passes"]["full"]["px"]
grp = {n: g for g, ns in vis["groups"].items() for n in ns}
iso = {n: vis["passes"][grp[n]]["px"][n] for n in full}


def wbox(o):
    bb = [o.matrix_world @ Vector(c) for c in o.bound_box]
    mn = Vector(map(min, *bb)) if False else Vector((min(v.x for v in bb), min(v.y for v in bb), min(v.z for v in bb)))
    mx = Vector((max(v.x for v in bb), max(v.y for v in bb), max(v.z for v in bb)))
    return mn, mx


sig = collections.defaultdict(list)
for o in objs:
    mn, mx = wbox(o)
    c, d = (mn + mx) / 2, mx - mn
    sig[(len(o.data.polygons), round(c.x, 4), round(c.y, 4), round(c.z, 4), round(d.x, 4), round(d.y, 4), round(d.z, 4))].append(o)
DUP = set()
for k, os_ in sig.items():
    if len(os_) > 1:
        keep = sorted(os_, key=lambda o: (-full.get(o.name, [0])[0], o.name))[0]
        DUP.update(o.name for o in os_ if o is not keep)
        # coincident copies z-fight in the visibility renders: the kept copy can lose the depth test to its own
        # twin in one pass (140T belt: full 6,125 px, own group 0 px -> both copies ended in REVIEW). It inherits the
        # best pixel counts of its set.
        for D in (full, iso):
            vals = [D[o.name] for o in os_ if o.name in D]
            if vals and keep.name in D:
                D[keep.name] = [max(v[i] for v in vals) for i in range(len(vals[0]))]
DUP |= {n for n in getattr(cfg, "EXTRA_DUPLICATES", ()) if n in full}
PLANNED = {o.name for o in objs}  # empty exploded parents get no review flags; their pieces carry them
full = {n: v for n, v in full.items() if n in PLANNED}
HID_A = {n for n in full if iso[n][0] == 0 and n not in DUP}
HID_B = {n for n in full if iso[n][0] > 0 and full[n][0] == 0 and n not in DUP}
BARELY = {n for n in full if 0 < full[n][0] <= 4 and n not in DUP} | {n for n in getattr(cfg, "EXTRA_BARELY", ()) if n in full}
HID_A -= BARELY
HID_B -= BARELY
# cfg.EXTRA_HIDDEN: object names the owner wants in REVIEW - Hidden whatever the visibility pass says
EXTRA_HIDDEN = {n for n in getattr(cfg, "EXTRA_HIDDEN", ()) if n in full}
HID_A |= EXTRA_HIDDEN
HID_B -= EXTRA_HIDDEN
BARELY -= EXTRA_HIDDEN
DUP -= EXTRA_HIDDEN
log("flags dup", len(DUP), "hidden", len(HID_A), "hidden-by-others", len(HID_B), "barely", len(BARELY))
for o in objs:
    CAD_BOX[o.name] = wbox(o)
# glTF exports can be unwelded polygon soup: every analyser that walks edges (tube walls, plate loops, hole
# plugging) then fails. cfg.PRE_WELD_M (e.g. 1e-6) welds every mesh first; shape-exact, custom normals survive.
if getattr(cfg, "PRE_WELD_M", 0):
    welded = sum(build.weld(me_, cfg.PRE_WELD_M) > 0 for me_ in {o.data for o in objs})
    log("pre-weld", cfg.PRE_WELD_M, "m:", welded, "meshes had loose duplicate vertices")
# cfg.MESH_PREP = {mesh name: fn(me)}: per-mesh edits of the welded CAD before any analysis, for owner rulings that
# change the look (MK5n: "you dont need decals on the swerve covers. you can just have it be a circle")
for mn_, fn_ in getattr(cfg, "MESH_PREP", {}).items():
    if mn_ in bpy.data.meshes:
        log("mesh prep", mn_, fn_(bpy.data.meshes[mn_]))
# cfg.FLOOR_Z pins the floor plane: Onshape "Origin Cube" markers hang up to 109 mm below the wheels
FLOOR_Z = getattr(cfg, "FLOOR_Z", None)
if FLOOR_Z is None:
    FLOOR_Z = min(wbox(o)[0].z for o in objs)
if getattr(cfg, "NO_FLOOR_ALL", False):  # library asset builds: an asset can be mounted any way up
    FLOOR_Z = -1e9
STATIC = set(vis["groups"]["main"])
# clear parts never cover or fill another part's holes (build.ray_hits passes through them)
build.SEE_THROUGH = {n for g in getattr(cfg, "SEE_THROUGH_GROUPS", ()) for n in vis["groups"].get(g, [])}
GROUP_NAMES = {g: set(ns) for g, ns in vis["groups"].items()}


# chain is thin and porous: it runs through slots (elevator top tube) without hiding them
CHAIN = {o.name for o in objs if cfg.is_chain(part(o), o.data)}
# cfg.HEAD_CYL_NO_OCCLUDE: parts rebuilt as one head cylinder (rivets) lose their shank and tail, so their CAD
# shank/tail must not hide holes either (tail-side rivet holes on a climber upright, seen through a clear panel,
# were dropped as hidden). A head lying on the wall still makes the hole "covered" (tube_hole_covered / hole_state).
# A set of part names limits it to those parts' hole tests (e.g. only the climber uprights: elsewhere the rivet head
# cylinders cover the holes on the visible side, so keeping them only adds triangles).
_FM = getattr(cfg, "FORCE_METHOD", {})
_HCN = getattr(cfg, "HEAD_CYL_NO_OCCLUDE", False)
HEADCYL = {o.name for o in objs if _FM.get(o.data.name, _FM.get(part(o))) == "head cylinder"} if _HCN else set()


def occluders_for(o):
    occ = (STATIC | GROUP_NAMES.get(grp.get(o.name, "main"), set())) - CHAIN
    if HEADCYL and (_HCN is True or part(o) in _HCN):
        occ = occ - HEADCYL
    return occ


log("floor z", round(FLOOR_Z, 4))
vis_stats = collections.Counter()

# ------------------------------------------------------------------ method assignment
L = align.LIB
LIB_RULES = cfg.LIB_RULES
REV_KEYS = ("gear", "pulley", "sprocket", "bearing", "spacer", "washer", "rivet", "bhcs", "shcs", "standoff",
            "stand-off", "hub", "wheel", "ring", "shield", "ball", "pc tube", "silicon wrap", "surgical tube pulley",
            "shaft", "wcp-1712", "wcp-1713", "wcp-1723", "wcp-1727", "wcp-1728", "wcp-0233", "wcp-0432", "wcp-1740",
            "wcp-0037", "wcp-1735", "wcp-1734", "wcp-1405", "wcp-0228", "wcp-0156", "coupler") + tuple(getattr(cfg, "REV_KEYS_EXTRA", ()))
# REV_KEYS_EXTRA: robot-specific round-part names the generic keywords miss ("Socket Screw", "Wing Nut", 垫片...)


LIMELIGHTS = [("Limelight 3/Limelight 3.blend", "Limelight 3"), ("Limelight 4/Limelight 4.blend", "Limelight 4")]
# assets a robot can force per part with LIMELIGHT_PARTS = {part: asset}; the 3G shares the Limelight 3's CAD, so a fit
# cannot tell them apart and it is never picked automatically
LIMELIGHT_ASSETS = {a: (f, a) for f, a in LIMELIGHTS + [("Limelight 3G/Limelight 3G.blend", "Limelight 3G")]}
KEEP_AS_IS = cfg.KEEP_AS_IS
REBUILD_KEEP_ALL_HOLES = cfg.REBUILD_KEEP_ALL_HOLES


def mat_rings_for(p):
    """cfg.LATHE_MATERIAL_RINGS: True/False, or a callable(part name) -> bool. Owner 2026-10-08: "for all bearings
    you can use one color" -> lambda p: "bearing" not in p.lower() (bearings one colour, other lathes keep colour rings)."""
    v = getattr(cfg, "LATHE_MATERIAL_RINGS", True)
    return bool(v(p)) if callable(v) else v


def flip_matrix(spec):
    """LIB_FLIP / LIB_ASSEMBLIES "flip": turns about the asset's own axes, "X" / "Y" / "Z" = 180 deg, "Z90" = a quarter
    turn; a "+" sequence such as "X+Z90" applies the LAST token first (the Pigeon fitted upside down and, once
    flipped, with its mounting holes on the other diagonal)."""
    F = np.eye(3)
    for tok in str(spec).split("+"):
        tok = tok.strip()
        F = F @ np.array(Matrix.Rotation(math.radians(float(tok[1:]) if len(tok) > 1 else 180.0), 3, tok[0]))
    return F


def lib_rule(p, me):
    for prefix, f, a in LIB_RULES:
        if p == prefix:
            # several CAD pieces can share a motor's name (connectors, pinions): only fit the body
            ld = mesh_diag_mm(lib_mesh(f, a))
            if 0.6 * ld < mesh_diag_mm(me) < 1.6 * ld:
                return f, a
    return None


def has_key(pl, keys):
    """Keyword at a word start ("ring" must not match "spring")."""
    return any(re.search(r"(?<![a-z])" + re.escape(k), pl) for k in keys)


def mesh_diag_mm(me):
    co, _, _ = geo.mesh_arrays(me)
    return float(np.linalg.norm(np.ptp(co, axis=0)) / MM)


plan = {}  # mesh name -> dict(method, new_mesh, info)
lib_cache = {}


def lib_mesh(f, a):
    """Evaluated library asset mesh in library world space (modifiers applied)."""
    key = (f, a)
    if key in lib_cache:
        return lib_cache[key]
    ob = align.append_object(f"{L}/{f}", a)
    sc.collection.objects.link(ob)
    d2 = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(d2), preserve_all_data_layers=True, depsgraph=d2)
    me.transform(ob.matrix_world)
    sc.collection.objects.unlink(ob)
    bpy.data.objects.remove(ob)
    me.name = a + " [library]"
    build.fix_inverted_shells(me)  # the Minion asset's shaft is an inside-out closed shell
    lib_cache[key] = me
    return me


KRAKEN_ASSET = getattr(cfg, "KRAKEN_ASSET", ("WCP-0940 (Kraken X60)/WCP-0940 (Kraken X60).blend", "WCP-0940 (Kraken X60)"))


_CAP_PROBE = []


def kraken_cap_probe():
    """Library-space sample points on the faces of the kraken cap that fill the library Kraken's open notch (more than
    1.5 mm from the library body), cached."""
    if not _CAP_PROBE:
        tj = json.load(open(os.path.join(HERE, "assets", "kraken_cap_x60.json")))
        V = np.array(tj["verts"], float)
        tri = np.array([(f[0], f[i], f[i + 1]) for f in tj["faces"] for i in range(1, len(f) - 1)])
        pts, _ = geo.sample_surface(V, tri, 3000)
        kco, ktri, _ = geo.mesh_arrays(lib_mesh(*KRAKEN_ASSET))
        bvh = BVHTree.FromPolygons([Vector(c_) for c_ in kco], [tuple(t_) for t_ in ktri])
        _CAP_PROBE.append(np.array([q for q in pts if bvh.find_nearest(Vector(q))[3] > 1.5e-3]))
    return _CAP_PROBE[0]


CAP_TOL = 1.5e-3  # sample point counts as covered within this distance of CAD surface (library fits are ~1 mm)


def cad_cap_coverage(o, Mlib):
    """Share of the cap's notch-filling faces lying on original CAD surface, for the library Kraken placed on object o
    through Mlib (library space -> o's mesh space). Any CAD mesh near the motor counts: the cap can be fused into the
    body mesh or exported as its own piece. A capless notch with connector pins stays low."""
    P = kraken_cap_probe()
    if len(P) == 0:
        return 0.0
    Mw = o.matrix_world @ Matrix(Mlib)
    Pw = np.array([tuple(Mw @ Vector(q)) for q in P])
    mn, mx = Pw.min(0) - 3e-3, Pw.max(0) + 3e-3
    dist = np.full(len(Pw), 1e9)
    for q in objs:
        bmn, bmx = CAD_BOX[q.name]
        if any(bmn[i] > mx[i] or bmx[i] < mn[i] for i in range(3)):
            continue
        co, tri, _ = geo.mesh_arrays(q.data)
        if len(tri) == 0:
            continue
        Mq = np.array(q.matrix_world)
        cw = co @ Mq[:3, :3].T + Mq[:3, 3]
        bvh = BVHTree.FromPolygons([Vector(c_) for c_ in cw], [tuple(t_) for t_ in tri])
        for i, p_ in enumerate(Pw):
            hit = bvh.find_nearest(Vector(p_), CAP_TOL)
            if hit[0] is not None:
                dist[i] = min(dist[i], hit[3])
    hit = dist <= CAP_TOL
    return float(hit.mean()), float(np.median(dist[hit])) if hit.any() else 9.0


def note_kraken_cap(entry, objs_md, M):
    """A library Kraken X60 was fitted with matrix M: record which of its objects had a CAD cap (they get one later)."""
    if not getattr(cfg, "KRAKEN_CAP", True):
        return
    caps = entry.setdefault("kraken_caps", {})
    cov = entry.setdefault("kraken_cap_coverage", {})
    for o in objs_md:
        share, med = cad_cap_coverage(o, M)
        cov[o.name] = round(share, 2)
        log("kraken cap check", o.name, "coverage", cov[o.name], "median", round(med * 1000, 2), "mm",
            "-> CAD cap found" if share >= 0.5 else "-> none")
        if share >= 0.5:
            caps[o.name] = [list(r_) for r_ in M]
            if med > 0.8e-3:  # off-angle library fit, or a differently shaped CAD cap: check the renders
                log("WARNING kraken cap: placed cap is", round(med * 1000, 2), "mm off the CAD cap on", o.name)


def kraken_material():
    """The library Kraken X60's body material (largest face area of the asset)."""
    km = lib_mesh(*KRAKEN_ASSET)
    area = collections.Counter()
    for q in km.polygons:
        area[q.material_index] += q.area
    return km.materials[area.most_common(1)[0][0]]


def revolve_axis(me):
    co, tri, _ = geo.mesh_arrays(me)
    pts, _ = geo.sample_surface(co, tri, 20000)
    best = (9, None, None)
    for d in geo.candidate_axes(pts):
        s, c = geo.revolve_score(pts, d)
        if s < best[0]:
            best = (s, d, c)
    return best


DENSE = getattr(cfg, "VIS_DENSE", False)


def _hole_points(cc, poly):
    """Hole centre, plus (cfg.VIS_DENSE) 6 points 60% of the way to the rim, so a slot seen through a narrow gap counts."""
    pts = [np.asarray(cc, float)]
    if DENSE and poly is not None and len(poly) >= 3:
        P_ = np.asarray(poly, float)
        for i in np.linspace(0, len(P_), 6, endpoint=False).astype(int):
            pts.append(pts[0] + 0.6 * (P_[i] - pts[0]))
    return pts


NO_FLOOR = set(getattr(cfg, "NO_FLOOR_OCCLUSION", ()))


def floor_for(o):
    """cfg.NO_FLOOR_OCCLUSION {part names}: the floor plane doesn't block hole-visibility rays for these parts, so
    holes on their undersides count as visible (chassis rails seen from below on a climbing robot)."""
    return -1e9 if part(o) in NO_FLOOR else FLOOR_Z


def tube_hole_visible(o, info, wall, H):
    L_, k, m, sgn = info["L"], wall["k"], wall["m"], wall["s"]
    nrm = np.zeros(3)
    nrm[k] = sgn
    M = o.matrix_world
    nw = (M.to_3x3() @ Vector(nrm)).normalized()
    for cc in _hole_points(H.mean(0), H):
        p = np.array(info["c"], float)
        p[L_], p[m] = cc[0], cc[1]
        p[k] = info["c"][k] + sgn * (info["half"][k] + 5e-4)
        if build.point_visible(sc, dg, M @ Vector(p), nw, occluders_for(o), floor_for(o), dense=DENSE):
            return True
    return False


def hole_flush(objs_, info, h0, h1, cc, r):
    """Something (bearing face, shaft end) lies in the plate's face plane over this hole: filling the hole would
    give two coplanar faces that z-fight."""
    d, u, w = info["d"], info["u"], info["w"]
    for o in objs_:
        M = o.matrix_world
        for lvl, sgn in ((h1, 1), (h0, -1)):
            hits = 0
            for s2 in build.hole_samples(cc, r, None):
                p = u * s2[0] + w * s2[1] + d * (lvl + sgn * 5e-4)
                ow = M @ Vector(p)
                dw = (M.to_3x3() @ Vector(-d * sgn)).normalized()
                if _flush_hit(o, ow, dw):
                    hits += 1
            if hits >= 3:
                return True
    return False


def plate_hole_visible(o, info, level_top, cc, poly=None):
    d, u, w = info["d"], info["u"], info["w"]
    h0 = info["dns"][0][0]
    M = o.matrix_world
    for c2 in _hole_points(cc, poly):
        base = u * c2[0] + w * c2[1]
        for lvl, sgn in ((level_top, 1), (h0, -1)):
            pw = M @ Vector(base + d * (lvl + sgn * 5e-4))
            nw = (M.to_3x3() @ Vector(d * sgn)).normalized()
            if build.point_visible(sc, dg, pw, nw, occluders_for(o), floor_for(o), dense=DENSE):
                return True
    return False


def ends_plugged(objs_, d, c, h0, h1, rin):
    u, w = geo.perp_basis(d)
    for o in objs_:
        for hstart, sgn in ((h1 + 5e-4, -1), (h0 - 5e-4, 1)):
            hits = 0
            for k in range(4):
                a = k * math.pi / 2 + 0.3
                p = c + (u * math.cos(a) + w * math.sin(a)) * 0.5 * rin + d * hstart
                if build.ray_hits(sc, dg, o, p, d * sgn, 3.5e-3):
                    hits += 1
            if hits < 3:
                return False
    return True


def bore_flush(objs_, d, c, h0, h1, rin):
    """A shaft/bolt end lying in the end plane of the bore: capping the bore there gives two coplanar faces that
    z-fight (flicker in Unity). Rays start 0.5 mm outside each end and look inward through the bore."""
    u, w = geo.perp_basis(d)
    for o in objs_:
        M = o.matrix_world
        for hstart, sgn in ((h1 + 5e-4, -1), (h0 - 5e-4, 1)):
            for fr in (0.5, 0.8):  # two radii: a hollow shaft (tapped end, tube) misses the inner ring
                hits = 0
                for k in range(4):
                    a = k * math.pi / 2 + 0.3
                    p = c + (u * math.cos(a) + w * math.sin(a)) * fr * rin + d * hstart
                    ow = M @ Vector(p)
                    dw = (M.to_3x3() @ Vector(d * sgn)).normalized()
                    if _flush_hit(o, ow, dw):
                        hits += 1
                if hits >= 3:
                    return True
    return False


def _flush_hit(o, ow, dw):
    """True if a *different* object has a face in the plane 0.5 mm along dw (looks past o's own coplanar face)."""
    p, left = ow, 7.5e-4
    for _ in range(3):
        hit, loc, nrm, idx, hob, _ = sc.ray_cast(dg, p, dw, distance=left)
        if not hit or hob is None:
            return False
        dist = (loc - ow).length
        if dist > 7.5e-4:
            return False
        if hob.name != o.name:
            return abs(dist - 5e-4) < 2.5e-4
        p = loc + dw * 2e-6
        left = 7.5e-4 - (p - ow).length
    return False


def _flush_same(o, ow, dw, nw):
    """A different object's face 0.5 mm along dw that faces like nw: a cap there would z-fight with it."""
    p, left = ow, 7.5e-4
    for _ in range(4):
        hit, loc, nrm, idx, hob, _ = sc.ray_cast(dg, p, dw, distance=left)
        if not hit or hob is None:
            return False
        dist = (loc - ow).length
        if dist > 7.5e-4:
            return False
        if hob.name != o.name and abs(dist - 5e-4) < 2.5e-4 and nrm.dot(nw) > 0.9:
            return True
        p = loc + dw * 2e-6
        left = 7.5e-4 - (p - ow).length
        if left <= 0:
            return False
    return False


def flush_hole_keeper(objs_):
    """keep() for build.plug_small_holes: leave a hole open when another part's face lies in the rim plane inside
    it, facing the same way (a boss or flange pressed flush into the hole: WCP-0379 in WCP-0374, the elevator
    gearbox spacers). The plug cap would sit exactly on that face."""
    def keep(rims):
        for pts, n in rims:
            c = pts.mean(0)
            r = float(np.linalg.norm(pts - c, axis=1).mean())
            u, w = geo.perp_basis(n)
            for o in objs_:
                M = o.matrix_world
                nw = (M.to_3x3() @ Vector(n)).normalized()
                hits = 0
                for k in range(6):
                    a = k * math.pi / 3 + 0.2
                    q = c + (u * math.cos(a) + w * math.sin(a)) * 0.5 * r + n * 5e-4
                    if _flush_same(o, M @ Vector(q), -nw, nw):
                        hits += 1
                if hits >= 3:
                    return True
        return False
    return keep


def all_hole_keeper(objs_):
    """cfg.DECIMATE_KEEP_HOLES = "all": keep every hole 3.5 mm or more across (no bolt or visibility test), as the
    owner approved them; flush inserts keep theirs as always."""
    flush = flush_hole_keeper(objs_)

    def keep(rims):
        if flush(rims):
            return True
        if len(rims) < 2:
            return False
        p0 = rims[0][0]
        return 2 * float(np.linalg.norm(p0 - p0.mean(0), axis=1).mean()) >= 3.5e-3
    return keep


def fallback_hole_keeper(objs_):
    """For a plate that fell back to decimation: holes under 8 mm are plugged unless the plate rule keeps them
    (3.5 mm or more across, nothing in them, visible from outside). Flush inserts always keep their hole."""
    flush = flush_hole_keeper(objs_)

    def keep(rims):
        if flush(rims):
            return True
        if len(rims) < 2:
            return False
        (p0, n0), (p1, n1) = rims[0], rims[1]
        c0, c1 = p0.mean(0), p1.mean(0)
        r = float(np.linalg.norm(p0 - c0, axis=1).mean())
        ax = c1 - c0
        L = float(np.linalg.norm(ax))
        if 2 * r < 3.5e-3 or L < 1e-6:
            return False
        ax = ax / L
        u, w = geo.perp_basis(ax)
        for o in objs_:
            starts = [c0 - ax * 5e-4 + (u * math.cos(a) + w * math.sin(a)) * 0.4 * r for a in (0.3, 2.4, 4.5)]
            if sum(build.ray_hits(sc, dg, o, q, ax, L + 1e-3) for q in starts) >= 2:
                continue  # a bolt or shaft is in it
            M = o.matrix_world
            for c, n in ((c0, n0), (c1, n1)):
                if build.point_visible(sc, dg, M @ Vector(c + n * 5e-4), M.to_3x3() @ Vector(n), occluders_for(o), FLOOR_Z):
                    return True
        return False
    return keep


def end_flush(objs_, d, c, h0, h1, rin, rout):
    """Per end: another part's face lies in the end plane over the face annulus (bearing pressed flush into a
    plate or block). Returns (start, end) booleans."""
    u, w = geo.perp_basis(d)
    r = rin + 0.6 * (rout - rin)
    out = [False, False]
    for o in objs_:
        M = o.matrix_world
        for j, (hstart, sgn) in enumerate(((h0 - 5e-4, 1), (h1 + 5e-4, -1))):
            hits = 0
            for k in range(6):
                a = k * math.pi / 3 + 0.2
                p = c + (u * math.cos(a) + w * math.sin(a)) * r + d * hstart
                ow = M @ Vector(p)
                dw = (M.to_3x3() @ Vector(d * sgn)).normalized()
                if _flush_hit(o, ow, dw):
                    hits += 1
            if hits >= 3:
                out[j] = True
                continue
            # a same-facing face in the end plane just outside the rim: the part sits flush in a bore, and a
            # simplified bore polygon (decimated plate) would overlap the end face (Elev. GB vs Part1-01 bearing)
            nw = (M.to_3x3() @ Vector(-d * sgn)).normalized()
            hits = 0
            for k in range(6):
                a = k * math.pi / 3 + 0.2
                p = c + (u * math.cos(a) + w * math.sin(a)) * (rout + 7.5e-4) + d * hstart
                if _flush_same(o, M @ Vector(p), (M.to_3x3() @ Vector(d * sgn)).normalized(), nw):
                    hits += 1
            if hits >= 3:
                out[j] = True
    return out


def bore_open(objs_, d, c, h0, h1, rin):
    # rays along the axis through the bore at half the bore radius: open if nothing else is in the way
    u, w = geo.perp_basis(d)
    for o in objs_:
        hits = 0
        for k in range(4):
            a = k * math.pi / 2 + 0.3
            p = c + (u * math.cos(a) + w * math.sin(a)) * 0.5 * rin + d * (h1 + 1e-3)
            if build.ray_hits(sc, dg, o, p, -d, (h1 - h0) + 2e-3):
                hits += 1
        if hits >= 2:
            return False
    return True


def hex_bore_test(objs_):
    """cfg.HEX_THROUGH_BORE: build.hex_candidate asks whether a through-bore of a plain hex prism is open in the
    robot (nothing else along its axis, same test as lathe bores)."""
    if not getattr(cfg, "HEX_THROUGH_BORE", False):
        return None
    return lambda d_, c_, h0_, h1_, r_: bore_open(objs_, d_, c_, h0_, h1_, r_)


stats = collections.Counter()
for n_done, (mname, objs_md) in enumerate(sorted(users.items(), key=lambda kv: kv[0])):
    if n_done % 100 == 0:
        log("planning", n_done, "/", len(users), dict(stats))
    me = bpy.data.meshes[mname]
    rep = objs_md[0]
    p = part(rep)
    pl = p.lower()
    entry = {"method": None, "objects": [o.name for o in objs_md]}
    plan[mname] = entry
    try:
        is_tube = cfg.is_tube(p, me)
        if any(o.name in user_done for o in objs_md) and not is_tube and p not in REBUILD_KEEP_ALL_HOLES:
            entry["method"] = "keep (your modifiers)"
            continue
        if p in KEEP_AS_IS:
            entry["method"] = "unchanged (already simplified)"
            continue
        if build.tri_count(me) <= 30:
            entry["method"] = "unchanged (already light)"  # rebuilding a 12-tri box only breaks it
            continue
        if p in cfg.LIMELIGHT_PARTS:
            dco, dtri, _ = geo.mesh_arrays(me)
            best_ll = None
            want = cfg.LIMELIGHT_PARTS.get(p) if isinstance(cfg.LIMELIGHT_PARTS, dict) else None
            for f_, a_ in ([LIMELIGHT_ASSETS[want]] if want else LIMELIGHTS):
                lm = lib_mesh(f_, a_)
                sco, stri, _ = geo.mesh_arrays(lm)
                R, t, ch = align.align(sco, stri, dco, dtri)
                if best_ll is None or sum(ch) < sum(best_ll[3]):
                    best_ll = (lm, R, t, ch, a_)
            lm, R, t, ch, a_ = best_ll
            if sum(ch) / 2 < 3.0e-3:
                new = lm.copy()
                new.name = a_ + " [library]"
                new.transform(align.to_matrix(R, t))
                new.update()
                entry.update(method="library", new=new, chamfer_mm=round(sum(ch) / 2 * 1000, 2), asset=a_)
                continue
            entry["note"] = f"Limelight fit too loose ({sum(ch) / 2 * 1000:.1f}mm)"
        combo = getattr(cfg, "LIB_COMBOS", {}).get(p)
        if combo:  # gearbox + motor merged into one CAD mesh: two library assets fitted together (libcombo.py)
            new, info_ = libcombo.fit_combo(me, [lib_mesh(f_, a_) for f_, a_ in combo])
            if new is not None:
                new.name = p + " [library combo]"
                M_mot = info_.pop("motor_matrix")
                entry.update(method="library", new=new, asset=" + ".join(a_ for _, a_ in combo), **info_)
                if combo[1][1] == KRAKEN_ASSET[1]:
                    note_kraken_cap(entry, objs_md, M_mot)
                continue
            info_.pop("motor_matrix", None)
            entry["note"] = f"library combo rejected: {info_}"
        ident = getattr(cfg, "LIB_IDENTITY", {}).get(p)
        if ident:  # custom asset built in this part's own mesh frame: no fit
            if len(ident) > 2 and ident[2] == "collection":
                if ident not in lib_cache:
                    lib_cache[ident] = libcombo.collection_mesh(f"{L}/{ident[0]}", ident[1])
                new = lib_cache[ident].copy()
            else:
                new = lib_mesh(*ident[:2]).copy()
            if len(ident) > 3 and ident[3]:  # asset centred in the part's frame: shift back by that centre
                new.transform(Matrix.Translation(Vector(ident[3])))
            new.name = ident[1] + " [library]"
            entry.update(method="library", new=new, asset=ident[1], note="custom asset built in this part's frame")
            # owner: motors are never inside a module asset -> library motors as their own objects (added after the
            # swaps); a Kraken X60 mount gets the cap check like any other X60
            mounts = getattr(cfg, "LIB_IDENTITY_MOTORS", {}).get(p, ())
            if mounts:
                entry["motor_mounts"] = [list(m_) for m_ in mounts]
                for f_, a_, lab_, M_ in mounts:
                    if a_ == KRAKEN_ASSET[1]:
                        note_kraken_cap(entry, objs_md, Matrix(M_))
            continue
        rule = lib_rule(p, me)
        if rule and build.tri_count(lib_mesh(*rule)) >= build.tri_count(me):
            entry["note"] = "library asset is heavier than the CAD part; not swapped"
            rule = None
        if rule:
            lm = lib_mesh(*rule)
            sco, stri, _ = geo.mesh_arrays(lm)
            dco, dtri, _ = geo.mesh_arrays(me)
            R, t, ch = align.align(sco, stri, dco, dtri)
            flip = getattr(cfg, "LIB_FLIP", {}).get(p)
            if flip:  # 180 deg about the asset's own axis through its centre (symmetric fits; set after a visual check)
                F = flip_matrix(flip)  # or a "+" sequence such as "X+Z90"
                c0 = 0.5 * (sco.min(0) + sco.max(0))
                t = t + R @ (c0 - F @ c0)
                R = R @ F
            if sum(ch) / 2 < getattr(cfg, "LIB_MAX_MM", {}).get(p, 3.0) * 1e-3:  # per-part limit after a visual check
                new = lm.copy()
                new.name = rule[1] + " [library]"
                new.transform(align.to_matrix(R, t))
                nudge = getattr(cfg, "LIB_NUDGE_MM", {})  # {mesh or part name: world (dx, dy, dz) mm} after a visual check
                dv = nudge.get(mname, nudge.get(p))
                if dv:
                    wv = Vector(dv) * MM  # world offset -> the mesh's local frame (instances share it; set it per mesh)
                    new.transform(Matrix.Translation(objs_md[0].matrix_world.to_3x3().inverted() @ wv))
                    entry["lib_nudge_mm"] = list(dv)
                new.update()
                entry.update(method="library", new=new, chamfer_mm=round(sum(ch) / 2 * 1000, 2), lib_asset=rule[1],
                             lib_matrix=[list(r_) for r_ in align.to_matrix(R, t)])
                if p in getattr(cfg, "LIB_CAD_COLOURS", ()):  # these library parts keep the CAD's colours
                    new.name = rule[1] + " [library, CAD colours]"
                    entry["cad_colours"] = build.recolour_from_cad(me, new)
                if rule[1] == KRAKEN_ASSET[1]:
                    note_kraken_cap(entry, objs_md, align.to_matrix(R, t))
                continue
            entry["note"] = f"library fit too loose ({sum(ch) / 2 * 1000:.1f}mm), decimated instead"
        if is_tube:
            # tubes modelled at an angle inside their own mesh are analysed in their own (axis-aligned) frame
            frame = build.tube_frame(me)
            tme, inst = me, objs_md
            if frame is not None:
                tme = me.copy()
                tme.transform(Matrix(frame.tolist()).to_4x4())
                back = Matrix(frame.T.tolist()).to_4x4()
                inst = [build.Framed(o, back) for o in objs_md]
            info = build.analyse_tube(tme)
            if info:
                keep, tot, cov, hid = {}, 0, 0, 0
                for wi, wall in enumerate(info["walls"]):
                    keep[wi] = []
                    for H in wall["holes"]:
                        tot += 1
                        fi = getattr(cfg, "TUBE_FILL_IRREGULAR", False)  # True, or a set of mesh / part names
                        fi = (mname in fi or p in fi) if isinstance(fi, (set, frozenset)) else fi
                        if fi and build.hole_is_engraving(H):
                            cov += 1  # engraved script / logo read as a wall opening: filled, not cut
                            vis_stats["tube engravings filled"] += 1
                            continue
                        if getattr(cfg, "TUBE_THROUGH_TEST", False) and build.tube_hole_is_pocket(tme, info, wall, H):
                            cov += 1  # a shallow engraved pocket outline, not a through hole: wall stays solid
                            vis_stats["tube pockets kept solid"] += 1
                            continue
                        if all(build.tube_hole_covered(sc, dg, o, info, wall, H) for o in inst):
                            cov += 1
                            continue
                        if not any(tube_hole_visible(o, info, wall, H) for o in inst):
                            vis_stats["tube holes not visible"] += 1
                            hid += 1
                            continue
                        keep[wi].append(H)
                split = 0
                for wi in keep:  # round holes traced as one loop -> back to separate circles
                    hs2, extra = [], []
                    for H in keep[wi]:
                        cs = build.split_merged_hole(H)
                        if cs:
                            extra += cs
                        else:
                            hs2.append(H)
                    cen = [np.asarray(H).mean(0) for H in hs2]
                    for C in extra:  # a recovered circle may also be listed on its own
                        if all(np.linalg.norm(C.mean(0) - c_) > 1e-3 for c_ in cen):
                            hs2.append(C); cen.append(C.mean(0)); split += 1
                    keep[wi] = hs2
                if split:
                    entry["split_holes"] = split
                new = build.build_tube(p + " [tube]", info, keep)
                dropped = 0
                if build.open_edges(new):
                    # holes the wall triangulation could not cut leave open edges (see-through cracks): rebuild once
                    # without the holes next to them; patching the gaps (fix_t_junctions) left internal faces instead
                    bm_ = bmesh.new(); bm_.from_mesh(new)
                    bad = np.array([v.co[:] for e in bm_.edges if len(e.link_faces) == 1 for v in e.verts]); bm_.free()
                    keep2 = {}
                    for wi, hs in keep.items():
                        wall = info["walls"][wi]
                        keep2[wi] = []
                        for H in hs:
                            P3 = np.zeros((len(H), 3))
                            P3[:, info["L"]], P3[:, wall["m"]] = np.asarray(H)[:, 0], np.asarray(H)[:, 1]
                            P3[:, wall["k"]] = info["c"][wall["k"]] + wall["s"] * info["half"][wall["k"]]
                            near = np.min(np.linalg.norm(bad[:, None, :] - P3[None, ::max(1, len(P3) // 64), :], axis=2)) < 1e-3
                            if near:
                                dropped += 1
                            else:
                                keep2[wi].append(H)
                    new2 = build.build_tube(p + " [tube]", info, keep2) if dropped else None
                    if new2 is not None and build.open_edges(new2) == 0:
                        bpy.data.meshes.remove(new)
                        new, keep = new2, keep2
                    else:
                        if new2 is not None:
                            bpy.data.meshes.remove(new2)
                        dropped = 0
                        build.fix_t_junctions(new)  # T-junctions only: split and weld
                if build.open_edges(new) == 0:
                    # closed but partly flipped (e.g. the walls of its obround cut-outs): wind it outward
                    bm_ = bmesh.new(); bm_.from_mesh(new)
                    if any(len(e.link_faces) == 2 and not e.is_contiguous for e in bm_.edges):
                        bmesh.ops.recalc_face_normals(bm_, faces=bm_.faces)
                        bm_.to_mesh(new); new.update()
                    bm_.free()
                if frame is not None:
                    new.transform(back)
                    new.update()
                for m_ in me.materials:
                    new.materials.append(m_)
                entry.update(method="tube rebuild", new=new, holes=tot, covered_holes=cov, hidden_holes=hid, rotated=frame is not None)
                if dropped:
                    entry["dropped_holes"] = dropped
                    entry["note"] = f"{dropped} holes the wall triangulation could not cut were left closed"
                    log("tube", p, "dropped", dropped, "holes next to open edges")
            if tme is not me:
                bpy.data.meshes.remove(tme)
            if info:
                continue
        if cfg.is_chain(p, me) and mesh_diag_mm(me) < 30:
            # CAD exports either whole links or loose side plates / pins
            kind = build.chain_piece_kind(me)
            if kind == "plate":
                new = build.build_chain_plate(p + " [link plate]", me)
            elif kind == "pin":
                new = build.build_chain_pin(p + " [link pin]", me)
            else:
                new = build.build_chain_link(p + " [link]", me)
            if new is not None:
                build.transfer_materials(me, new)
                entry.update(method="chain link rebuild", new=new)
                continue
            dims_ = np.sort(np.ptp(geo.mesh_arrays(me)[0], axis=0))
            new = build.build_obb_box(p + " [box]", me, shrink_mid=0.95 if dims_[0] < 9e-3 else 1.0)
            build.transfer_materials(me, new)
            entry.update(method="box (chain link)", new=new)
            continue
        fm = getattr(cfg, "FORCE_METHOD", {})
        forced = fm.get(mname, fm.get(p))  # mesh datablock name first: generic part names ("Part 1") repeat
        if forced == "keep":
            entry.update(method="unchanged (already light)", note="kept (cfg.FORCE_METHOD)")
            continue
        if forced == "decimate":
            entry.update(method="decimate", note="forced (cfg.FORCE_METHOD)")
            continue
        if forced == "hex":  # rounded-hex stock: exact 6-sided prism (owner rule), skipping the roundness test
            hx, dev, ap = build.hex_candidate(me, journals=not getattr(cfg, "FORCE_HEX_PLAIN", False), bore_test=hex_bore_test(objs_md))
            if hx is not None:
                hx.name = p + " [hex]"
                build.transfer_materials(me, hx)
                entry.update(method="hex prism (6 sides)", new=hx, hex_dev_mm=round(dev, 2), note="forced (cfg.FORCE_METHOD)")
                if "hex_bore_mm" in hx:
                    entry["hex_bore_mm"] = round(float(hx["hex_bore_mm"]), 2)
                continue
        if forced == "head cylinder":  # (owner) rivets: one closed cylinder for the head, shank and tail dropped
            s_, d_, c_ = revolve_axis(me)
            new = build.build_head_cylinder(p + " [head]", me, d_, c_)
            if new is not None:
                build.transfer_materials(me, new)
                entry.update(method="cylinder rebuild", new=new, note="head cylinder (cfg.FORCE_METHOD)")
                continue
        if forced == "sectioned":  # hex + round lengths along a shaft, each its own closed prism (with its bore)
            new = build.build_sectioned(p + " [sectioned]", me)
            if new is not None:
                build.transfer_materials(me, new)
                entry.update(method="sectioned (hex + round)", new=new, sections=build.build_sectioned.sections,
                             note="forced (cfg.FORCE_METHOD)")
                continue
        builder = getattr(cfg, "BUILDERS", {}).get(forced) if isinstance(forced, str) else None
        if builder:  # robot-specific remodel builders: cfg.BUILDERS {method: fn(part, mesh) -> (mesh|None, info)}
            new, info_ = builder(p, me)
            if new is not None:
                build.transfer_materials(me, new)
                entry.update({k: v for k, v in info_.items() if k != "method"}, new=new,
                             method=info_.get("method", forced), note=f"cfg.BUILDERS {forced}")
                continue
            entry["note"] = f"{forced} failed: {info_.get('error', info_)}"
        no_round = forced == "plate"  # flat ring with tabs: no hex/lathe (the lathe takes the tab radius)
        if forced == "kraken cap":
            # the CAD's Kraken connector cap: its outline extruded, in the library Kraken's body colour
            new = build.build_cap_prism("kraken cap", me)
            new.materials.append(kraken_material())
            entry.update(method="kraken cap (custom)", new=new)
            continue
        if forced in ("section lathe", "stacked lathe", "smooth lathe"):
            s_, d_, c_ = revolve_axis(me)
            if forced == "section lathe":  # cups and open hubs: union of axial cross-sections keeps the cavity
                new = build.build_revolve_sections(p + " [cyl]", me, d_, c_)
            elif forced == "stacked lathe":  # shafts with hex and round lengths
                new = build.build_stacked_lathe(p + " [cyl]", me, d_, c_)
            else:  # nubbed treads: a coarse profile drops the fake ring grooves of the nub rows
                new = build.build_revolve(p + " [cyl]", me, d_, c_, True, toothed=True, tol_mm=4.0)
            if new is not None:
                (build.transfer_materials if forced == "stacked lathe" else build.transfer_materials_rings)(me, new)
                entry.update(method="cylinder rebuild", new=new, round_score=round(s_, 4), note=forced + " (cfg.FORCE_METHOD)")
                continue
        if p in getattr(cfg, "WHEEL_SECTION", ()):
            new, err = build.build_wheel_section(p + " [wheel]", me)
            if new is not None and build.tri_count(new) < build.tri_count(me):
                build.transfer_materials(me, new)
                entry.update(method="wheel section", new=new, err_mm=[round(err[0], 2), round(err[1], 2)])
                continue
            if new is not None:
                bpy.data.meshes.remove(new)
        if p in getattr(cfg, "EYEBOLTS", ()):
            new = build.build_eyebolt(p + " [eyebolt]", me)
            if new is not None:
                build.transfer_materials(me, new)
                entry.update(method="eyebolt rebuild", new=new)
                continue
        if cfg.SPRINGS.get(p) == "helix":
            new = build.build_spring(p + " [spring]", me)
            if new is not None:
                build.transfer_materials(me, new)
                entry.update(method="spring rebuild", new=new)
                continue
        if cfg.is_belt(p):
            new = build.build_belt(p + " [belt]", me)
            if new is not None and build.tri_count(new) >= build.tri_count(me):
                bpy.data.meshes.remove(new)  # already a plain loop in the CAD
                new = None
                entry["note"] = "belt rebuild not lighter; kept"
            if new is not None:
                build.transfer_materials(me, new)
                entry.update(method="belt rebuild", new=new)
                continue
        if getattr(cfg, "SPHERES", False):  # balls: low-poly UV sphere
            sc_, sr_, sspread = build.sphere_fit(me)
            # a ball is as deep as it is wide: a flat disc's rim also lies on a sphere (MK5n turret decal discs)
            _ext = np.ptp(np.array([v.co for v in me.vertices]), axis=0)
            if sspread < 0.02 and _ext.min() > 0.8 * _ext.max():
                new = build.build_sphere(p + " [ball]", sc_, sr_)
                build.transfer_materials(me, new)
                entry.update(method="sphere", new=new)
                continue
        s, d, c = revolve_axis(me)
        hex_named = ("hex" in pl or "standoff" in pl or "stand-off" in pl or "spacer" in pl) and not any(
            x in pl for x in ("bore", "bearing", "hub", "pulley", "gear", "sprocket"))
        if not no_round and s > 0.02 and (hex_named or s < 0.09) and forced != "lathe":
            hx, dev, ap = build.hex_candidate(me, bore_test=hex_bore_test(objs_md))
            if hx is not None and dev < max(0.4, 0.04 * ap):
                hx.name = p + " [hex]"
                build.transfer_materials(me, hx)
                entry.update(method="hex prism (6 sides)", new=hx, hex_dev_mm=round(dev, 2))
                if "hex_bore_mm" in hx:
                    entry["hex_bore_mm"] = round(float(hx["hex_bore_mm"]), 2)
                continue
            if hx is not None:
                bpy.data.meshes.remove(hx)
        extra = p in getattr(cfg, "ROUND_EXTRA", ())
        # cfg.ROUND_BY_SHAPE: nameless round parts (custom modules) go by shape alone
        named = (has_key(pl, REV_KEYS) or extra or getattr(cfg, "ROUND_BY_SHAPE", False)) and "plate" not in pl
        if not no_round and (s < 0.015 or (named and s < 0.09) or forced in ("lathe-open", "lathe")):
            co, tri, _ = geo.mesh_arrays(me)
            toothed = (has_key(pl, ("gear", "pulley", "sprocket")) or extra
                       or any(k in pl for k in ("1712", "1713", "1723", "0233", "0432", "1740")) or s > 0.01)
            edges, Ro, Ri = build.revolve_profile(co, tri, d, c, toothed=toothed)
            ratio = Ri.min() / max(Ro.max(), 1e-9)
            length = edges[-1] - edges[0]
            keep_bore, flush = False, False
            if ratio > 0.3:
                elongated = length > 2.4 * Ro.max()
                if elongated and ends_plugged(objs_md, d, c, edges[0], edges[-1], Ri.min()):
                    keep_bore = False
                elif not elongated and ratio > 0.4:
                    keep_bore = True  # flat ring (ring gear, washer-like plate): the opening is part of its look
                else:
                    keep_bore = ratio > 0.55 or bore_open(objs_md, d, c, edges[0], edges[-1], Ri.min())
            if forced == "lathe-open":
                keep_bore = True
            elif Ri.min() > 1.5e-3 and bore_flush(objs_md, d, c, edges[0], edges[-1], Ri.min()):
                # a shaft/bolt end lies flush in the bore: leave it open at its largest radius (hex corners clear)
                keep_bore, flush = True, True
                entry["note"] = "bore kept open: flush shaft end"
            ins = end_flush(objs_md, d, c, edges[0], edges[-1], Ri.min() if keep_bore else 0.0, Ro.max())
            li = getattr(cfg, "LATHE_INSET", {}).get(p)  # forced inset (mm) of both end faces: flush inserts
            if li:                                         # sitting in a sprocket's windows escape end_flush
                ins = (li * 1e-3, li * 1e-3)
            new = build.build_revolve(p + " [cyl]", me, d, c, keep_bore, toothed=toothed,
                                      tol_mm=getattr(cfg, "LATHE_TOL_MM", {}).get(p),
                                      mat_rings=mat_rings_for(p), bore_max=flush,
                                      bore_steps=getattr(cfg, "FLUSH_BORE_KEEP_STEPS", False),
                                      inset=tuple((x if isinstance(x, float) else 1.5e-4) if x else 0.0 for x in ins),
                                      remodel=getattr(cfg, "LATHE_REMODEL", False))  # grooves -> one cylinder
            if any(ins):
                entry["end_inset"] = list(ins)
            if not keep_bore and Ri.min() > 1.5e-3:
                entry["_reopen"] = (d.copy(), c.copy(), edges[0], edges[-1], float(Ri.min()), toothed, tuple(ins))
            if build.tri_count(new) >= sum(len(q.vertices) - 2 for q in me.polygons):
                bpy.data.meshes.remove(new)
                entry["method"] = "unchanged (already light)"
                continue
            build.transfer_materials_rings(me, new, single=not mat_rings_for(p))
            entry.update(method="cylinder rebuild", new=new, bore=bool(keep_bore), round_score=round(s, 4))
            continue
        info = build.analyse_stepped(me)
        if info:
            xh = build.cross_axis_holes(me, np.asarray(info["d"]))
            if xh:
                entry["note"] = f"plate rebuild rejected ({len(xh)} holes across the plate axis), decimated instead"
                info = None
        if info:
            holes = build.plan_holes(info)
            drop, nf, ns = set(), 0, 0
            h0 = info["dns"][0][0]
            nv = 0
            for (k, idx, cc, req, Lp, match) in ([] if p in REBUILD_KEEP_ALL_HOLES else holes):
                if req * 2 < 3.5e-3:
                    ns += 1
                elif (req * 2 <= 20e-3 and build.hole_state(sc, dg, objs_md, info["u"], info["w"], info["d"], h0, k[0], cc, req, Lp)
                      and not hole_flush(objs_md, info, h0, k[0], cc, req)):
                    nf += 1
                elif req * 2 <= 20e-3 and not any(plate_hole_visible(o, info, k[0], cc, Lp) for o in objs_md):
                    nv += 1
                    vis_stats["plate holes not visible"] += 1
                else:
                    continue
                drop.add((k, idx))
                drop.update(match)
            hole_sides = getattr(cfg, "PLATE_HOLE_SIDES", None)  # optional r_mm -> sides for round plate holes
            chk = build.build_stepped(p + " [check]", info, set(), circle_sides=hole_sides)
            e1, e2 = build.fit_error(me, chk)
            new = build.build_stepped(p + " [plate]", info, drop, tol_mm=getattr(cfg, "PLATE_TOL", {}).get(me.name, getattr(cfg, "PLATE_TOL", {}).get(p, 0.45)),
                                      circle_sides=hole_sides)
            # dropping holes may only ADD flat face area; if it shrank, the drop removed material -> keep all holes
            dv = Vector(info["d"])
            cap = lambda m_: sum(q.area for q in m_.polygons if abs(q.normal.dot(dv)) > 0.99)
            if drop and cap(new) < 0.995 * cap(chk):
                bpy.data.meshes.remove(new)
                new, chk = chk, None
                new.name = p + " [plate]"
                entry["note"] = "hole dropping removed material; kept all holes"
                nf = ns = nv = 0
            elif chk is not None:
                bpy.data.meshes.remove(chk)
            build.transfer_materials(me, new)
            if (e1 < 1.5 and e2 < 4.0 and build.tri_count(new) < len(me.polygons) * 2 and build.open_edges(new) == 0
                    and build.backface_pct(new) <= build.backface_pct(me) + 1.0):
                entry.update(method="plate rebuild", new=new, filled_holes=nf, small_holes=ns, hidden_holes=nv, err_mm=[round(e1, 2), round(e2, 2)])
                continue
            entry["note"] = (f"plate rebuild rejected (err {e1:.1f}/{e2:.1f}mm, {build.open_edges(new)} open edges, "
                             f"{build.backface_pct(new):.0f}% back faces), decimated instead")
            bpy.data.meshes.remove(new)
        px, perr = build.build_profile_extrusion(p + " [profile]", me)
        if px is not None and perr < 1.0 and build.tri_count(px) < len(me.polygons):
            build.transfer_materials(me, px)
            entry.update(method="profile extrusion", new=px, err_mm=round(perr, 2))
            entry.pop("note", None)
            continue
        if px is not None:
            bpy.data.meshes.remove(px)
        entry["method"] = "decimate"
    except Exception as ex:
        import traceback
        traceback.print_exc()
        entry["method"] = "decimate"
        entry["note"] = f"error: {ex}"
    finally:
        stats[entry["method"]] += 1

log("planned", dict(stats))
log("visibility", dict(vis_stats))

# ------------------------------------------------------------------ multi-piece library swaps
# (before the per-mesh replacements, so the fit uses the CAD meshes)
ASM_TARGET, REPLACED = set(), set()
for spec, parts_ in ASM:
    sco_w, stri_w = [], []
    base = 0
    for o in parts_:
        co, tri = align.world_mesh_arrays(o)
        sco_w.append(co)
        stri_w.append(tri + base)
        base += len(co)
    dco, dtri = np.vstack(sco_w), np.vstack(stri_w)
    lm = lib_mesh(spec["lib"], spec["asset"])
    if build.tri_count(lm) >= sum(orig_tris.get(o.name, 0) for o in parts_):
        log(spec["label"], "library asset is heavier than the CAD pieces; kept")
        continue
    lco, ltri, _ = geo.mesh_arrays(lm)
    R, t, ch = align.align(lco, ltri, dco, dtri, tiebreak="bbox" if spec.get("bbox_tiebreak") else None)
    if spec.get("flip"):  # turn about the asset's own axis through its centre after a visual check: "X"/"Y"/"Z" =
        # 180 deg as LIB_FLIP (PDP / breaker fitted end-for-end), "Z90" / "Z270" = quarter turns (a near-square Pigeon
        # fitted 90 deg off: its mounting holes on the other diagonal)
        F = flip_matrix(spec["flip"])
        c0 = 0.5 * (lco.min(0) + lco.max(0))
        t = t + R @ (c0 - F @ c0)
        R = R @ F
    if sum(ch) / 2 >= spec.get("max_mm", 3.0) * 1e-3:  # "max_mm": per-assembly limit after a visual check
        log(spec["label"], "library fit too loose", round(sum(ch) / 2 * 1000, 2), "mm; pieces kept")
        continue
    new = lm.copy()
    new.name = spec["asset"] + " [library]"
    new.transform(align.to_matrix(R, t))
    if spec.get("target") == "largest":
        # swap in place on the biggest piece: its name, parent and transform stay
        ob = build.largest_piece(parts_)
        new.transform(ob.matrix_world.inverted())
        ob.modifiers.clear()
        ob.data = new
    else:
        asm = spec["anchor"](parts_)
        new.transform(asm.matrix_world.inverted())
        ob = bpy.data.objects.new(spec["object"], new)
        for c_ in parts_[0].users_collection:
            c_.objects.link(ob)
        ob.parent = asm
        ob.matrix_parent_inverse = Matrix.Identity(4)
        ob.matrix_basis = Matrix.Identity(4)
    ASM_TARGET.add(ob.name)
    rest = [o.name for o in parts_ if o is not ob]
    REPLACED.update(rest)
    plan["__asm__" + ob.name] = {"method": "library", "objects": [ob.name], "replaced": rest,
                                 "chamfer_mm": round(sum(ch) / 2 * 1000, 2), "asset": spec["asset"]}
    log(spec["label"], "->", ob.name, "replaces", len(parts_), "pieces, chamfer", round(sum(ch) / 2 * 1000, 2), "mm")
# LIB_EMPTY_IDENTITY: {empty name prefix: (blend, collection)} -- a collection asset built in an assembly EMPTY's
# own frame (a swerve module exported as an empty + ~133 CAD pieces): one new object "<empty> [library]" under the empty
# with identity transform; all the empty's CAD pieces -> REVIEW - Replaced by library.
OBJ_NAMES = {o.name for o in objs}
# a third element "object" names an OBJECT asset instead of a collection (evaluated like LIB_RULES assets); a
# fourth, a key-piece regex: the asset was built in that piece's frame (kit_extract.py; Onshape glTF assembly empties sit
# at the origin with every instance's pieces in world space), so each instance's object takes that piece's transform
for prefix, (f_, cname, *kind_) in getattr(cfg, "LIB_EMPTY_IDENTITY", {}).items():
    key_rx = kind_[1] if len(kind_) > 1 else None
    for emp in [o for o in sc.objects if o.type == 'EMPTY' and o.name.startswith(prefix)]:
        pieces = [o for o in emp.children_recursive if o.type == 'MESH' and o.name in OBJ_NAMES]
        key = ("__coll__", f_, cname, *kind_[:1])
        if key not in lib_cache:
            lib_cache[key] = (lib_mesh(f_, cname) if kind_ and kind_[0] == "object"
                              else libcombo.collection_mesh(f"{L}/{f_}", cname))
        keyo = None
        if key_rx:
            keyo = next(iter(sorted([o for o in pieces if re.search(key_rx, o.name)], key=lambda o: o.name)), None)
            if keyo is None:
                log(cname, "key piece", key_rx, "not found under", emp.name, "; kept")
                continue
        if build.tri_count(lib_cache[key]) >= sum(orig_tris.get(o.name, 0) for o in pieces):
            log(cname, "library asset is heavier than the CAD pieces of", emp.name, "; kept")
            continue
        new = lib_cache[key].copy()
        new.name = cname + " [library]"
        ob = bpy.data.objects.new(emp.name + " [library]", new)
        for c_ in (pieces[0].users_collection if pieces else emp.users_collection):
            c_.objects.link(ob)
        ob.parent = emp
        ob.matrix_parent_inverse = Matrix.Identity(4)
        ob.matrix_basis = Matrix.Identity(4) if keyo is None else emp.matrix_world.inverted() @ keyo.matrix_world
        ASM_TARGET.add(ob.name)
        REPLACED.update(o.name for o in pieces)
        plan["__asm__" + ob.name] = {"method": "library", "objects": [ob.name], "replaced": [o.name for o in pieces],
                                     "asset": cname, "note": "asset in the assembly's own frame" if keyo is None else
                                     f"asset in the frame of {keyo.name}"}
        log(cname, "->", ob.name, "replaces", len(pieces), "pieces")
for k, e in plan.items():
    if not k.startswith("__asm__"):
        e["objects"] = [n for n in e["objects"] if n not in ASM_TARGET]

# Motors exported as several meshes (Kraken body + connectors, rear cover, screws): the library asset stands for the
# whole motor, so CAD pieces with the motor's part name that sit inside a swapped body go to "REVIEW - Replaced by
# library" instead of poking through the asset.
LIB_BODIES = [on for e in plan.values() if e["method"] == "library" for on in e["objects"]]
CAD_CAPS = {on for e in plan.values() if e["method"] == "kraken cap (custom)" for on in e["objects"]}
for on in LIB_BODIES:
    if on not in CAD_BOX:
        continue  # a multi-piece swap that landed on a new object (LIB_ASSEMBLIES "anchor"): its pieces are handled above
    pb = cfg.part(on)
    mn, mx = CAD_BOX[on]
    for o in objs:
        if o.name in LIB_BODIES or o.name in REPLACED or o.name not in CAD_BOX or o.name in CAD_CAPS:
            continue
        if cfg.part(o.name) != pb and cfg.part(o.name) not in getattr(cfg, "LIB_ABSORB", {}).get(pb, ()):
            continue
        c = (CAD_BOX[o.name][0] + CAD_BOX[o.name][1]) / 2
        if all(mn[i] - 3e-3 <= c[i] <= mx[i] + 3e-3 for i in range(3)):
            REPLACED.add(o.name)
log("library leftovers moved to review:", len(REPLACED) - sum(len(e.get("replaced", [])) for e in plan.values()))

# ------------------------------------------------------------------ apply mesh replacements
for mname, e in plan.items():
    new = e.get("new")
    if new is None:
        continue
    if e["method"] != "library" and build.fix_inverted_shells(new):
        e["note"] = ((e.get("note") or "") + "; inside-out shell flipped").lstrip("; ")
    for on in e["objects"]:
        o = bpy.data.objects[on]
        if e["method"] == "tube rebuild" or o.name in user_done:
            o.modifiers.clear()  # the owner's Decimate stack would decimate the clean rebuild
        o.data = new
    e.pop("new")

# ------------------------------------------------------------------ library motors next to module assets
# cfg.LIB_IDENTITY_MOTORS = {part: [(lib .blend, asset, label, 4x4 mount matrix: library space -> the part's frame)]}:
# every object of that LIB_IDENTITY part gets each motor as a child object "<label> - <object>" (owner: "the motors should
# not be in the swerve asset. this goes for any module you simplify into an asset"). One mesh per mount, shared.
MOTORS_ADDED = []
for e in list(plan.values()):
    mounts = e.pop("motor_mounts", None)
    if not mounts:
        continue
    for f_, a_, lab_, M_ in mounts:
        mm = lib_mesh(f_, a_).copy()
        mm.transform(Matrix(M_))
        mm.name = a_ + " [library]"
        for on in e["objects"]:
            b = bpy.data.objects[on]
            ob = bpy.data.objects.new(f"{lab_} - {on}", mm)
            for c_ in b.users_collection:
                c_.objects.link(ob)
            ob.parent = b
            ob.matrix_parent_inverse = Matrix.Identity(4)
            ob.matrix_basis = Matrix.Identity(4)
            MOTORS_ADDED.append(ob.name)
    e["motors"] = [m_[2] for m_ in mounts]
if MOTORS_ADDED:
    plan["__identity_motors__"] = {"method": "library", "objects": list(MOTORS_ADDED),
                                   "note": "library motor as its own object next to a module asset (cfg.LIB_IDENTITY_MOTORS)"}
log("library motors added next to module assets:", len(MOTORS_ADDED))

# ------------------------------------------------------------------ Kraken caps
# The library Kraken X60 has an open notch at its back where the CTR connector cap sits. Owner: a motor gets a
# "kraken cap" (in the motor's colour) ONLY if its CAD had a cap there. Motors whose CAD cap was rebuilt (FORCE_METHOD
# "kraken cap") keep that; a library Kraken whose original CAD covered the cap's notch-filling faces (cad_cap_coverage,
# tested at planning time on every CAD mesh near the motor) gets a child "kraken cap" placed through its library
# alignment (assets/kraken_cap_x60.json, library space). cfg.KRAKEN_CAP = False turns this off.
CAPS_ADDED = []
if getattr(cfg, "KRAKEN_CAP", True):
    # motors whose CAD had a cap (planning-time check), and every library Kraken X60 fit for snapping CAD cap pieces
    bodies = [(bpy.data.objects[on], Matrix(m_)) for e in plan.values()
              for on, m_ in e.get("kraken_caps", {}).items() if on in e["objects"]]
    fits = [(bpy.data.objects[on], Matrix(e["lib_matrix"])) for e in plan.values()
            if e.get("lib_asset") == KRAKEN_ASSET[1] and "lib_matrix" in e for on in e["objects"]]
    fits += [bl for bl in bodies if bl[0].name not in {f_[0].name for f_ in fits}]
    capped = set()
    if fits and (bodies or CAD_CAPS):
        # assets/kraken_cap_x60.json is built from the library motor itself (kraken_cap_asset.py): through a motor's
        # library fit it sits flush in the motor's notch, flat side on the notch wall
        tj = json.load(open(os.path.join(HERE, "assets", "kraken_cap_x60.json")))
        cap_me = build.new_mesh_from("kraken cap", tj["verts"], tj["faces"],
                                     {i for i, f in enumerate(tj["smooth"]) if f}, {frozenset(e) for e in tj["sharp_edges"]})
        cap_me.materials.append(kraken_material())
        ctr = lambda o: o.matrix_world @ (sum((Vector(c_) for c_ in o.bound_box), Vector()) / 8)
        for on in CAD_CAPS:
            # owner: a cap fits its Kraken, never off-angle -> a CAD cap piece takes the library-fitted cap of its motor
            # (keeps its name, parent and transform); the CAD outline (build_cap_prism) stays only without a motor nearby
            cp = bpy.data.objects[on]
            b, Lb = min(fits, key=lambda bl: (ctr(bl[0]) - ctr(cp)).length)
            if (ctr(b) - ctr(cp)).length < 0.08:
                m_ = cap_me.copy()
                m_.transform(cp.matrix_world.inverted() @ b.matrix_world @ Lb)
                m_.update()
                cp.data = m_
                capped.add(b.name)
        for b, Lb in bodies:
            if b.name in capped:
                continue
            ob = bpy.data.objects.new("kraken cap", cap_me)
            for c_ in b.users_collection:
                c_.objects.link(ob)
            ob.parent = b
            ob.matrix_parent_inverse = Matrix.Identity(4)
            ob.matrix_basis = Lb
            CAPS_ADDED.append(ob.name)
        if CAPS_ADDED:
            plan["__kraken_caps__"] = {"method": "kraken cap (custom)", "objects": list(CAPS_ADDED), "note": "added on the library Kraken's rear notch"}
    log("kraken caps: CAD cap pieces", len(CAD_CAPS), "re-fitted to their motor", len(capped), "caps added", len(CAPS_ADDED))

# ------------------------------------------------------------------ flush shaft ends, re-tested on the new scene
# Planning tested the CAD. A library motor shaft or a rebuilt shaft end can end flush with a gear or pulley whose
# bore was capped, and the two coplanar faces z-fight in Unity: test again with the final meshes, reopen those.
dg.update()
reopened = 0
for mname, e in plan.items():
    args = e.pop("_reopen", None)
    if args is None or e["method"] != "cylinder rebuild" or not e["objects"]:
        continue
    d, c, h0, h1, rin, toothed, ins = args
    objs_ = [bpy.data.objects[on] for on in e["objects"]]
    if not bore_flush(objs_, d, c, h0, h1, rin):
        continue
    me = bpy.data.meshes[mname]
    old_new = objs_[0].data
    # never heavier than the CAD part: a 216-tri rounded-hex shaft came back as a 10,656-tri lathe (counterbore
    # steps along 533 mm) -> retry without the steps, else keep the capped bore
    new = None
    for steps in ([True, False] if getattr(cfg, "FLUSH_BORE_KEEP_STEPS", False) else [False]):
        if new is not None:
            bpy.data.meshes.remove(new)
        new = build.build_revolve(cfg.part(objs_[0].name) + " [cyl]", me, d, c, True, toothed=toothed,
                                  tol_mm=getattr(cfg, "LATHE_TOL_MM", {}).get(cfg.part(objs_[0].name)),
                                  mat_rings=mat_rings_for(cfg.part(objs_[0].name)), bore_max=True,
                                  bore_steps=steps, inset=tuple((x if isinstance(x, float) else 1.5e-4) if x else 0.0 for x in ins),
                                  remodel=getattr(cfg, "LATHE_REMODEL", False))
        if build.tri_count(new) < build.tri_count(me):
            break
    if build.tri_count(new) >= build.tri_count(me):
        log("bore reopen skipped (heavier than the CAD part):", objs_[0].name, build.tri_count(new), ">=", build.tri_count(me))
        bpy.data.meshes.remove(new)
        e["note"] = (e.get("note", "") + "; flush shaft end found after the swaps, open bore heavier than the CAD: kept capped").lstrip("; ")
        continue
    build.transfer_materials_rings(me, new, single=not mat_rings_for(cfg.part(objs_[0].name)))
    build.fix_inverted_shells(new)  # the reopened 1.03" cone spacer came out inside-out
    for o in objs_:
        o.data = new
    if old_new.users == 0:
        bpy.data.meshes.remove(old_new)
    e.update(bore=True, note="bore kept open: flush shaft end (found after the swaps)")
    reopened += 1
log("bores reopened for flush shaft ends after the swaps:", reopened)

# ------------------------------------------------------------------ decimate (your modifier stack)
import shading


def collapse_eval(me, ratio, kept=()):
    """Collapse-decimate a copy: (p99 two-sided distance mm, back-face hit %, p95 distance mm of the kept holes'
    points to the result). Distance alone let Collapse flip faces (Talon logo, bellmouth) and turn the holes a
    part keeps into triangles (swerve plates)."""
    tmp = bpy.data.objects.new("__collapse_probe", me)
    sc.collection.objects.link(tmp)
    m = tmp.modifiers.new("Decimate", 'DECIMATE')
    m.decimate_type = 'COLLAPSE'
    m.ratio = ratio
    d2 = bpy.context.evaluated_depsgraph_get()
    d2.update()
    dm = bpy.data.meshes.new_from_object(tmp.evaluated_get(d2), depsgraph=d2)
    bpy.data.objects.remove(tmp)
    e1, e2 = build.fit_error(me, dm, 4000, q=99)
    bf = build.backface_pct(dm)
    k95 = 0.0
    if kept and len(dm.polygons):
        from mathutils.bvhtree import BVHTree
        t = BVHTree.FromPolygons([v.co for v in dm.vertices], [tuple(q.vertices) for q in dm.polygons])
        k95 = float(np.percentile([t.find_nearest(q)[3] for q in kept], 95)) / MM
    bpy.data.meshes.remove(dm)
    return max(e1, e2), bf, k95


def add_decimate(o, ratio):
    if ratio is not None:
        m = o.modifiers.new("Decimate", 'DECIMATE')
        m.decimate_type = 'COLLAPSE'
        m.ratio = ratio
    shading.add_shading(o)


for mname, e in plan.items():
    if e["method"] != "decimate":
        continue
    me = bpy.data.meshes.get(mname)
    if me is None:
        continue
    tris0 = sum(len(p.vertices) - 2 for p in me.polygons)
    target = int(np.clip(8.0 * mesh_diag_mm(me), 200, getattr(cfg, "DECIMATE_MAX_TARGET", 1000)))
    if target / max(tris0, 1) >= getattr(cfg, "DECIMATE_LIGHT_FRAC", 0.85):  # already light: unchanged, holes and all
        e["method"] = "unchanged (already light)"
        continue
    # plug small holes + merge coplanar faces first (shape-exact), then Collapse only if still needed
    # a plate that fell back to decimation keeps every hole the plate rule would keep (>3.5 mm), unless Collapse
    # can't reach its target with them (a swerve plate stayed at 12k tris): then plug at the usual 8 mm
    fallback = "plate rebuild rejected" in (e.get("note") or "")
    normals = "vote" if e["objects"] and cfg.part(e["objects"][0]) in getattr(cfg, "NORMALS_PARITY", ()) else "recalc"
    objs_e = [bpy.data.objects[on] for on in e["objects"] if on in bpy.data.objects]
    # a plate that fell back to decimation keeps the holes the plate rule keeps (3.5 mm+, empty, visible)
    # cfg.DECIMATE_KEEP_HOLES: every decimated part keeps those holes (owner: leave the sheet-metal holes)
    kh = getattr(cfg, "DECIMATE_KEEP_HOLES", False)
    if isinstance(kh, (set, frozenset)):  # only these mesh / part names keep their visible empty holes
        kh = mname in kh or bool(e["objects"] and cfg.part(e["objects"][0]) in kh)
    elif isinstance(kh, dict):  # {mesh / part: True (as the set form) | "all" (every hole 3.5 mm+, this part only)}
        kh = kh.get(mname, kh.get(cfg.part(e["objects"][0])) if e["objects"] else None) or False
    keeper =(all_hole_keeper if kh == "all" else fallback_hole_keeper if fallback or kh else flush_hole_keeper)(objs_e)
    dr = getattr(cfg, "DECIMATE_RATIO", {})  # mesh datablock name first (generic part names repeat), then part name
    fixed = dr.get(mname, dr.get(cfg.part(e["objects"][0]))) if e["objects"] else dr.get(mname)
    kept = []
    plugged, tris = build.prep_for_decimate(me, plug_max=8e-3, normals=normals, keep=keeper, kept=kept,
                                            weld_dist=getattr(cfg, "MERGE_DIST", None))
    e["holes_plugged"] = plugged
    kept = build.round_hole_points(me)  # every round hole still open (kept small ones and larger ones up to 80 mm)
    if kept:
        e["hole_points_kept"] = len(kept)
    ratio = target / max(tris, 1)
    ratio = None if ratio >= 0.85 else max(round(ratio, 3), 0.05)
    big = mesh_diag_mm(me) > 150
    if big and tris > 400:
        ratio = min(ratio or 1.0, 0.2)
    # quality check: Collapse can tear thin or noisy CAD (encoder housings, motor covers) into spikes, flip faces,
    # or chew kept holes into triangles at the target count; raise the ratio until none of that happens
    tol_mm = max(0.5, 0.012 * mesh_diag_mm(me))
    if big:
        tol_mm = min(tol_mm, 1.0)  # large parts start low, so hold them to an absolute 1 mm
    e["decimate_err_mm"] = None
    if fixed is not None:
        ratio = fixed if fixed < 0.7 else None
        if ratio is not None:  # fixed ratios skip the loop, but record what they do
            err, bf, k95 = collapse_eval(me, ratio, kept)
            e.update(decimate_err_mm=round(err, 2), backface_pct=[round(build.backface_pct(me), 1), round(bf, 1)])
    bf0 = build.backface_pct(me) if ratio is not None and fixed is None else 0.0
    # hidden / replaced parts only need the distance check (orientation and holes don't show)
    in_review = all(on in DUP or on in HID_A or on in HID_B or on in BARELY or on in REPLACED for on in e["objects"])
    ratio0, tried = ratio, []
    bf_lim = bf0 + max(1.0, 0.15 * bf0)
    while ratio is not None and fixed is None:
        err, bf, k95 = collapse_eval(me, ratio, kept)
        tried.append((ratio, err, bf, k95))
        e["decimate_err_mm"] = round(err, 2)
        e["backface_pct"] = [round(bf0, 1), round(bf, 1)]
        e["kept_hole_err_mm"] = round(k95, 2) if kept else None
        # kept holes may go hex-like (0.35 mm on a 4-5 mm hole), not triangular (~1 mm)
        looks_ok = in_review or (bf <= bf_lim and k95 <= 0.35)
        if err <= tol_mm and looks_ok:
            break
        if err <= tol_mm and ratio * 1.5 > 3 * ratio0:
            # the extra triangles did not fix it (faces flipped by the hole plugs, not by Collapse: swerve plate
            # WCP-1110-002 stayed at ~1.5% from 0.15 to 0.32): take the cheapest ratio that is no worse
            ok = [t for t in tried if t[1] <= tol_mm]
            ratio, err, bf, k95 = min(ok, key=lambda t: (round(max(0.0, t[2] - bf_lim) * 2) / 2,
                                                         round(max(0.0, t[3] - 0.35), 1), t[0]))
            e.update(decimate_err_mm=round(err, 2), backface_pct=[round(bf0, 1), round(bf, 1)])
            e["note"] = ((e.get("note") or "") + "; orientation/hole checks capped, cheapest no-worse ratio kept").lstrip("; ")
            break
        ratio = round(ratio * 1.5, 3)
        if ratio >= 0.85:
            ratio = None
    e["ratio"] = ratio
    for on in e["objects"]:
        o = bpy.data.objects.get(on)
        if o:
            add_decimate(o, ratio)

# ------------------------------------------------------------------ final shading pass (owner's step)
# Limited Dissolve (<=5 deg) as the last geometry step; live Collapse objects get it as a Decimate (Planar)
# modifier. Then Smooth by Angle -> WeightedNormal on parts whose n-gons would shade with marks: decimated
# and untouched CAD parts (their custom normals are gone after the dissolve) + cfg.SHADE_EXTRA.
SHADE_METHODS = {"decimate", "unchanged (already light)", "spring rebuild", "wheel section"}  # wheel treads are 32-64-gons
method_of_obj = {on: e["method"] for e in plan.values() for on in e["objects"]}
lost_normals, done_me = set(), set()
shade_stats = collections.Counter()
for o in [o for o in sc.objects if o.type == 'MESH']:
    if method_of_obj.get(o.name) == "keep (your modifiers)":
        continue  # the owner's own parts stay exactly as they are: no dissolve, their shading stack untouched
    if any(m.type == 'DECIMATE' and m.decimate_type == 'COLLAPSE' for m in o.modifiers):
        shading.add_planar_dissolve(o)
    elif o.data.name not in done_me:
        done_me.add(o.data.name)
        if "custom_normal" in o.data.attributes:
            lost_normals.add(o.data.name)
        # cfg.LATHE_DISSOLVE_DEG (e.g. 1.0): at 5 deg shallow chamfer/cone facets of lathe parts merge into
        # non-planar n-gons that shade with patches; 1 deg keeps them (+~2% tris on one robot, so it is opt-in)
        lathe_like = method_of_obj.get(o.name) in ("cylinder rebuild", "hex prism (6 sides)")
        ang = getattr(cfg, "LATHE_DISSOLVE_DEG", shading.DISSOLVE_DEG) if lathe_like else shading.DISSOLVE_DEG
        # cfg.DISSOLVE_DEG_PARTS {part: deg}: gently curved sheets whose 5 deg n-gons cut their chords up to
        # 0.7 mm into a neighbour's surface (z-fighting); phase3.py honours the same angles
        ang = getattr(cfg, "DISSOLVE_DEG_PARTS", {}).get(part(o), ang)
        # cfg.BUILDERS remodels come out watertight and dissolved; the 0.05 mm Merge by Distance tore them
        # (sliver walls, 5 um wall pads), so they get the Limited Dissolve only
        remodelled = str(method_of_obj.get(o.name, "")).startswith("remodel")
        shading.limited_dissolve(o.data, angle_deg=ang, weld=None if remodelled else getattr(cfg, "MERGE_DIST", None))
    p = part(o)
    # cfg.SHADE_LATHE: lathe rebuilds get the stack too (remodelled lathes showed faceted bands, cavity off)
    want = (method_of_obj.get(o.name) in SHADE_METHODS or o.data.name in lost_normals
            or (getattr(cfg, "SHADE_LATHE", False) and method_of_obj.get(o.name) == "cylinder rebuild")
            # hex prisms with round stubs or a bore: the round faces were flat-shaded (60 deg hex edges stay sharp)
            or (getattr(cfg, "SHADE_LATHE", False) and method_of_obj.get(o.name) == "hex prism (6 sides)" and len(o.data.polygons) > 8)
            or p in getattr(cfg, "SHADE_EXTRA", ())) and p not in getattr(cfg, "SHADE_SKIP", ())
    if want:
        shade_stats["shaded"] += shading.add_shading(o) or 0
    else:
        shading.remove_shading(o)
log("final pass: dissolved", len(done_me), "meshes;", dict(shade_stats))

# ------------------------------------------------------------------ library materials: viewport colours + dedupe
new_mats = [m for m in bpy.data.materials if m.name not in mats_before]
by_base = collections.defaultdict(list)
for m in new_mats:
    by_base[re.sub(r"\.\d{3}$", "", m.name)].append(m)


def principled(m):
    if m.use_nodes and m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled':
                return n
    return None


mat_report = []
for base, ms in by_base.items():
    keep = sorted(ms, key=lambda m: (m.name != base, m.name))[0]
    for m in ms:
        if m is not keep:
            m.user_remap(keep)
            bpy.data.materials.remove(m)
    if keep.name != base and base not in bpy.data.materials:
        keep.name = base
    pb = principled(keep)
    if pb is not None:
        bc = pb.inputs['Base Color']
        if not bc.is_linked:
            keep.diffuse_color = (*bc.default_value[:3], 1.0)
        keep.metallic = pb.inputs['Metallic'].default_value
        keep.roughness = pb.inputs['Roughness'].default_value
    mat_report.append({"material": keep.name, "viewport_rgb": [round(x, 3) for x in keep.diffuse_color[:3]],
                       "principled": pb is not None, "metallic": round(keep.metallic, 2)})

# ------------------------------------------------------------------ review collections
def review_coll(name, tag):
    c = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    if c.name not in sc.collection.children:
        sc.collection.children.link(c)
    c.color_tag = tag
    return c


C = {
    "dup": review_coll("REVIEW - Duplicates", 'COLOR_01'),
    "a": review_coll("REVIEW - Hidden", 'COLOR_02'),
    "b": review_coll("REVIEW - Hidden behind other mechanisms", 'COLOR_03'),
    "c": review_coll("REVIEW - Barely visible", 'COLOR_05'),
}
if REPLACED:
    C["lib"] = review_coll("REVIEW - Replaced by library", 'COLOR_04')
# parts the simplification itself hides (cfg.REVIEW_AFTER: mesh datablock name -> reason), e.g. mecanum
# roller cores sealed inside the rebuilt (closed) roller skins
AFTER = {n for mn_, why in getattr(cfg, "REVIEW_AFTER", {}).items() for n in plan.get(mn_, {}).get("objects", [])}
if AFTER:
    C["after"] = review_coll("REVIEW - Hidden after simplification", 'COLOR_06')
# an added Kraken cap goes wherever its motor goes (a duplicate motor's cap would z-fight with the kept one's)
for on in CAPS_ADDED + MOTORS_ADDED:  # follow their parent into any REVIEW collection
    par = bpy.data.objects[on].parent.name
    for S in (REPLACED, DUP, HID_A, HID_B, BARELY):
        if par in S:
            S.add(on)
flag_of = {}
for key, names in (("lib", REPLACED), ("dup", DUP), ("a", HID_A), ("b", HID_B), ("c", BARELY), ("after", AFTER)):
    for n in names:
        if n in flag_of:
            continue
        o = bpy.data.objects.get(n)
        if o is None:
            continue
        flag_of[n] = C[key].name
        for c_ in list(o.users_collection):
            c_.objects.unlink(o)
        C[key].objects.link(o)

# ------------------------------------------------------------------ report + save
dg = bpy.context.evaluated_depsgraph_get()
dg.update()
final = {}
for o in sc.objects:
    if o.type == 'MESH':
        final[o.name] = eval_tris(o)
method_of = {}
for mname, e in plan.items():
    for on in e["objects"]:
        method_of[on] = e
rows = []
for on, n in final.items():
    e = method_of.get(on, {})
    rows.append({"object": on, "part": cfg.part(on), "method": e.get("method", "new"), "before": orig_tris.get(on),
                 "after": n, "review": flag_of.get(on), **{k: v for k, v in e.items() if k not in ("objects", "method")}})
tot_after = sum(final.values())
tot_review = sum(n for on, n in final.items() if on in flag_of)
json.dump({"before": sum(orig_tris.values()), "after": tot_after, "after_excluding_review": tot_after - tot_review,
           "review_tris": tot_review, "rows": rows, "materials": mat_report,
           "flags": {k: len(v) for k, v in (("replaced_by_library", REPLACED), ("dup", DUP), ("hidden", HID_A), ("hidden_by_others", HID_B), ("barely", BARELY))}},
          open(REPORT, "w"), indent=1)
log("tris before", sum(orig_tris.values()), "after", tot_after, "after excluding review", tot_after - tot_review)
# Library assets link their finishes from RobotMaterials.blend: make every linked datablock local so the output does
# not depend on where the asset library lives (one robot carried 6 linked materials, another 13)
n_local = build.make_all_local()
log("made local:", n_local, "linked datablocks")
# material slots no face uses are dropped: single-material lathes (LATHE_MATERIAL_RINGS False) kept the
# seal slot on every bearing with 0 faces, and the FBX carries it as an empty material. Owner's own parts and objects with
# object-linked slots are left alone.
n_slots, keep_me = 0, {o.data.name for o in sc.objects if o.type == 'MESH' and method_of_obj.get(o.name) == "keep (your modifiers)"}
obj_linked = {o.data.name for o in sc.objects if o.type == 'MESH' and any(s_.link == 'OBJECT' for s_ in o.material_slots)}
for me_ in bpy.data.meshes:
    if me_.users == 0 or me_.library or me_.name in keep_me or me_.name in obj_linked or len(me_.materials) < 2:
        continue
    mi = np.zeros(len(me_.polygons), np.int32); me_.polygons.foreach_get("material_index", mi)
    used = sorted(set(np.clip(mi, 0, len(me_.materials) - 1).tolist()))
    if len(used) == len(me_.materials):
        continue
    remap = np.zeros(len(me_.materials), np.int32); remap[used] = np.arange(len(used))
    for i in reversed(range(len(me_.materials))):
        if i not in used:
            me_.materials.pop(index=i); n_slots += 1
    me_.polygons.foreach_set("material_index", remap[np.clip(mi, 0, len(remap) - 1)]); me_.update()
log("unused material slots dropped:", n_slots)
bpy.ops.wm.save_as_mainfile(filepath=OUT, copy=False)
log("saved", OUT)
