# Low-poly camera as a library asset, built piece by piece from the CAD camera (e.g. OV2311 USB board camera and the
# 29 mm industrial camera + C-mount lens), in the style of the team's electronics assets: one mesh object at the origin
# (CAD local axes, centred), RobotMaterials finishes, CAD reference in an excluded collection, electronics catalog.
#   ROBOT=<robot> blender -b "<robot> - cleaned.blend" --python cam_asset.py -- <cad object> <out.blend> "<asset name>" <drop_mm> <work dir>
# Per CAD piece (mesh island): smaller than drop_mm -> left out (SMD parts, tiny screws); round -> lathe (owner's side
# tiers); flat with holes -> stepped plate (holes >= 2 mm kept); otherwise Merge by Distance + Limited Dissolve, then a
# Collapse whose ratio is raised until the p99 error is <= 0.2 mm (+1% of the piece size). Faces keep the nearest
# CAD colour, mapped to the closest RobotMaterials finish (CAM_MAT_OVERRIDE for hand-picked ones).
import bpy, bmesh, sys, os, math, json
import numpy as np
from mathutils import Vector, Matrix
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build, geo, libcombo, explode, align, shading
CAD, OUT, NAME, DROP_MM, WD = sys.argv[sys.argv.index("--") + 1:][:5]
DROP_MM = float(DROP_MM)
os.makedirs(WD, exist_ok=True)
import paths  # noqa: E402
RM = paths.robot_materials()
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"  # same catalog as the other electronics assets
MM = 1e-3
OVR = json.loads(os.environ.get("CAM_MAT_OVERRIDE", "{}"))      # CAD material name -> library material
FORCE = json.loads(os.environ.get("CAM_FORCE", "{}"))           # piece key -> lathe | lathe-open | sections | box | plate | split | keep | drop
KEEP_ONLY = set(json.loads(os.environ.get("CAM_KEEP_ONLY", "[]")))  # top-level pieces to keep (all others dropped)
EXCL = set(json.loads(os.environ.get("CAM_NO_UNION", "[]")))     # piece keys never merged into a coaxial lathe
UNION = os.environ.get("CAM_UNION") == "1"
# CAM_RATIO: {piece key: Collapse ratio to start from} -- the owner's "smooth from a few feet" bar on a big curved
# shell (printed camera hood): pick the ratio by render error against the CAD, the error loop can still raise it
RATIO = json.loads(os.environ.get("CAM_RATIO", "{}"))
# CAM_LENS: {"case": key, "lens": [keys], "plug_mm": d}: a board camera inside a case. The case's lens opening is
# plugged (holes < plug_mm) and every lens piece is cut at the case's front face: only what sticks out stays (owner).
LENS = json.loads(os.environ.get("CAM_LENS", "{}"))
PARTS = {}  # one lathe per coaxial group (blurs colour bands: off by default)
CANDS = ["Plastic - Black", "Anodized Aluminum - Black", "Plastic - Grey", "Plastic - Light Grey", "Plastic - White",
         "Aluminum - Machined", "Steel - Plain", "Plastic - Red", "LED - Green", "Plastic - CANYellow", "REV - Orange",
         "Anodized Aluminum - Blue", "Camera Lens", "Anodized Aluminum - Gold"]
with bpy.data.libraries.load(RM, link=False) as (src, dst):
    dst.materials = sorted(set(CANDS) | (set(OVR.values()) - {"CAD"}))
LIBM = {m.name: m for m in dst.materials if m}


def principled(m):
    if m and m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled':
                return n
    return None


def colour(m):
    pb = principled(m)
    return np.array(pb.inputs['Base Color'].default_value[:3]) if pb else np.array(m.diffuse_color[:3])


def lib_for(m):
    if m.name in OVR and OVR[m.name] != "CAD":
        return LIBM[OVR[m.name]]
    c = np.sqrt(np.clip(colour(m), 0, 1))  # compare in a roughly perceptual space
    best = min((n for n in CANDS if n != "Camera Lens"), key=lambda n: np.linalg.norm(np.sqrt(np.clip(colour(LIBM[n]), 0, 1)) - c))
    return LIBM[best]


cad = bpy.data.objects[CAD]
src = cad.data.copy()
# CAM_EXTRA: [{"object": name, "islands": [k...], "role": "asset" | "occluder"}] -- islands of other objects, moved into
# this camera's local frame: "asset" pieces become part of the asset (a printed case), "occluder" pieces only hide
# things (the mount bracket behind the board) and are not modelled.
EXTRA = json.loads(os.environ.get("CAM_EXTRA", "[]"))
occl = None
Minv = cad.matrix_world.inverted()
for ex in EXTRA:
    eo = bpy.data.objects[ex["object"]]
    EA = explode.MeshArrays(eo.data)
    eisl = sorted(explode.islands(eo.data), key=lambda f: -len(f))
    for ki in ex["islands"]:
        pm_ = explode.piece_mesh(EA, eisl[ki], f"extra~{ki}")
        pm_.transform(Minv @ eo.matrix_world)
        if ex["role"] == "asset":
            src = libcombo.join_meshes(src.name, [(src, Matrix.Identity(4)), (pm_, Matrix.Identity(4))])
        else:
            occl = pm_ if occl is None else libcombo.join_meshes("occl", [(occl, Matrix.Identity(4)), (pm_, Matrix.Identity(4))])
co0, _, _ = geo.mesh_arrays(src)
centre = 0.5 * (co0.min(0) + co0.max(0))
src.transform(Matrix.Translation(Vector(-centre)))
if occl is not None:
    occl.transform(Matrix.Translation(Vector(-centre)))
# Merge by Distance first (owner): welds the CAD's face patches so islands are real parts
bm = bmesh.new(); bm.from_mesh(src)
bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
bm.to_mesh(src); bm.free()
if "custom_normal" in src.attributes:
    src.attributes.remove(src.attributes["custom_normal"])
A = explode.MeshArrays(src)
isl = sorted(explode.islands(src), key=lambda f: -len(f))
from mathutils.bvhtree import BVHTree
_sco, _stri, _ = geo.mesh_arrays(src)
if occl is not None:
    _oco, _otri, _ = geo.mesh_arrays(occl)
    _stri = np.vstack([_stri, _otri + len(_sco)]); _sco = np.vstack([_sco, _oco])
BVH = BVHTree.FromPolygons([Vector(v) for v in _sco], [tuple(int(i) for i in t) for t in _stri])
_k = np.arange(32) + 0.5
_ph = np.arccos(1 - 2 * _k / 32); _th = np.pi * (1 + 5 ** 0.5) * _k
DIRS = [Vector(v) for v in np.c_[np.cos(_th) * np.sin(_ph), np.sin(_th) * np.sin(_ph), np.cos(_ph)]]


def visible_frac(pm, n=150):
    """Share of surface sample points on this piece from which some ray escapes the whole camera (owner: model only
    what can be seen; electronics enclosed by the shell are left out)."""
    co, tri, _ = geo.mesh_arrays(pm)
    if len(tri) == 0:
        return 0.0
    pts, _ = geo.sample_surface(co, tri, n, np.random.default_rng(1))
    vis = 0
    for p in pts:
        P = Vector(p)
        for d in DIRS:
            hit = BVH.ray_cast(P + d * 2e-5, d, 1.0)
            if hit[0] is None:
                vis += 1
                break
    return vis / len(pts)

parts, log = [], []


def tri_count(me):
    return sum(len(p.vertices) - 2 for p in me.polygons)


def collapse(pm, tol_mm, start=None):
    """Merge by Distance + Limited Dissolve, then Collapse; raise the ratio until p99 error <= tol."""
    bm = bmesh.new(); bm.from_mesh(pm)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
    bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(5), use_dissolve_boundaries=False, verts=bm.verts, edges=bm.edges,
                             delimit={'MATERIAL'})
    bmesh.ops.triangulate(bm, faces=bm.faces)
    bm.to_mesh(pm); bm.free()
    n0 = tri_count(pm)
    ratio = start or min(1.0, max(60, 6 * math.sqrt(n0)) / max(n0, 1))
    while True:
        if ratio >= 0.85:
            return pm.copy(), 1.0, 0.0
        tmp = bpy.data.objects.new("__c", pm); bpy.context.scene.collection.objects.link(tmp)
        md = tmp.modifiers.new("d", 'DECIMATE'); md.decimate_type = 'COLLAPSE'; md.ratio = ratio
        d2 = bpy.context.evaluated_depsgraph_get(); d2.update()
        dm = bpy.data.meshes.new_from_object(tmp.evaluated_get(d2), depsgraph=d2)
        bpy.data.objects.remove(tmp)
        e1, e2 = build.fit_error(pm, dm, 3000, q=99)
        if max(e1, e2) <= tol_mm:
            return dm, ratio, max(e1, e2)
        bpy.data.meshes.remove(dm)
        ratio *= 1.5


LATHE_CANDS = []


def round_axis(co, tri):
    pts, _ = geo.sample_surface(co, tri, 8000)
    best = (9, None, None)
    for d in geo.candidate_axes(pts):
        sc_, c = geo.revolve_score(pts, d)
        if sc_ < best[0]:
            best = (sc_, d, c)
    return best


def section_split(pm):
    """Cut a piece along its long axis where the cross-section changes between square and round (camera body +
    lens neck welded into one piece of one colour). Returns sub-meshes (faces by centroid)."""
    co, tri, _ = geo.mesh_arrays(pm)
    a = int(np.argmax(np.ptp(co, 0))); o1, o2 = [i for i in range(3) if i != a]
    edges = np.arange(co[:, a].min(), co[:, a].max() + 5e-4, 5e-4)
    kind = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        P = co[(co[:, a] >= lo) & (co[:, a] < hi)]
        if len(P) < 8:
            kind.append(None); continue
        cu, cw = 0.5 * (P[:, o1].min() + P[:, o1].max()), 0.5 * (P[:, o2].min() + P[:, o2].max())
        r = np.hypot(P[:, o1] - cu, P[:, o2] - cw).max()
        half = 0.5 * max(np.ptp(P[:, o1]), np.ptp(P[:, o2]))
        kind.append("sq" if r / max(half, 1e-9) > 1.2 else "rd")
    for i in range(len(kind)):  # fill empty bins from the left neighbour
        if kind[i] is None: kind[i] = kind[i - 1] if i else next((k for k in kind if k), "rd")
    w = 6  # majority over a 6 mm window: holes, chamfers and lettering must not cut the body
    kind = ["sq" if sum(k == "sq" for k in kind[max(0, i - w): i + w + 1]) * 2 > len(kind[max(0, i - w): i + w + 1]) else "rd"
            for i in range(len(kind))]
    cuts, cur = [], kind[0]
    for i, k in enumerate(kind):
        if k != cur:
            cuts.append(edges[i]); cur = k
    cuts = [c for c in cuts if c - edges[0] > 3e-3 and edges[-1] - c > 3e-3]
    if not cuts:
        return [pm.copy()]
    lt = np.empty(len(pm.polygons), np.int32); pm.polygons.foreach_get("loop_total", lt)
    cen = np.empty(len(pm.polygons) * 3); pm.polygons.foreach_get("center", cen); cen = cen.reshape(-1, 3)
    band = np.searchsorted(np.array(cuts), cen[:, a])
    sub = explode.MeshArrays(pm)
    return [explode.piece_mesh(sub, np.nonzero(band == b)[0], f"{pm.name}.s{b}") for b in np.unique(band)]


def lathe_build(pm, d, c, force, union=False):
    co, tri, _ = geo.mesh_arrays(pm)
    sc_ = 0.0
    edges, Ro, Ri = build.revolve_profile(co, tri, d, c)
    ratio = Ri.min() / max(Ro.max(), 1e-9)
    keep = union or force == "lathe-open" or ratio > 0.3
    new = build.build_revolve(pm.name + " [cyl]", pm, d, c, keep, tol_mm=max(0.2, (0.05 if Ro.max() > 10 * MM else 0.03) * Ro.max() / MM))
    # accept if the lathe surface lies on the CAD (one-sided: a union envelope skips internal faces)
    from mathutils.bvhtree import BVHTree
    sb = BVHTree.FromPolygons([Vector(v) for v in co], [tuple(int(i) for i in t) for t in tri])
    nco, ntri, _ = geo.mesh_arrays(new)
    P, _ = geo.sample_surface(nco, ntri, 3000)
    err = float(np.percentile([sb.find_nearest(Vector(q))[3] for q in P], 95)) / MM if len(P) else 9e9
    if tri_count(new) >= tri_count(pm) or (force not in ("lathe", "lathe-open") and err > 0.3 + 0.03 * Ro.max() / MM):
        bpy.data.meshes.remove(new)
        return None, err
    build.transfer_materials_rings(pm, new)
    return new, err


def aabb_box(name, pm):
    """Box on the CAD's own axes (most CAD parts are axis-aligned; PCA axes of a square chip are arbitrary)."""
    co, _, _ = geo.mesh_arrays(pm)
    lo, hi = co.min(0), co.max(0)
    verts = [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    me2 = build.new_mesh_from(name, verts, faces)
    build.fix_normals(me2)
    return me2


def box_try(pm, ext):
    best = None
    for mk in (lambda: aabb_box(pm.name + " [box]", pm), lambda: build.build_obb_box(pm.name + " [box]", pm)):
        cand = mk()
        e = max(build.fit_error(pm, cand, 3000))
        if best is None or e < best[1]:
            if best is not None:
                bpy.data.meshes.remove(best[0])
            best = (cand, e)
        else:
            bpy.data.meshes.remove(cand)
    box, e = best
    e1 = e2 = e
    if max(e1, e2) <= 0.25 + (0.08 if ext.max() < 10 else 0.03) * ext.max():
        build.transfer_materials(pm, box)
        return box, max(e1, e2)
    bpy.data.meshes.remove(box)
    return None, max(e1, e2)


def material_split(pm):
    """Connected pieces of one material each (a welded lens = barrel + knurled ring + glass + holder)."""
    bm = bmesh.new(); bm.from_mesh(pm); bm.faces.ensure_lookup_table()
    seen, groups = set(), []
    for f in bm.faces:
        if f.index in seen:
            continue
        stack, g = [f], []
        seen.add(f.index)
        while stack:
            x = stack.pop(); g.append(x.index)
            for e in x.edges:
                for y in e.link_faces:
                    if y.index not in seen and y.material_index == x.material_index:
                        seen.add(y.index); stack.append(y)
        groups.append(g)
    bm.free()
    sub = explode.MeshArrays(pm)
    return [explode.piece_mesh(sub, np.array(g), f"{pm.name}.{j}") for j, g in enumerate(sorted(groups, key=lambda g: -len(g)))]


def simplify_piece(pm, key, depth=0):
    """-> list of (mesh, record)."""
    co, tri, _ = geo.mesh_arrays(pm)
    ext = np.ptp(co, axis=0) / MM
    n0 = tri_count(pm)
    force = FORCE.get(key)
    rec = {"k": key, "tris": n0, "ext": [round(x, 1) for x in sorted(ext)]}
    if KEEP_ONLY and key.split(".")[0] not in KEEP_ONLY:
        rec["method"] = "dropped (not shell/lens)"
        return [(None, rec)]
    if force == "drop" or (force is None and ext.max() < DROP_MM):
        rec["method"] = "dropped"
        return [(None, rec)]
    if force is None:
        vf = visible_frac(pm)
        rec["visible"] = round(vf, 3)
        if vf == 0.0:
            rec["method"] = "hidden (enclosed)"
            return [(None, rec)]
    if force == "keep" or n0 <= 24:
        rec.update(method="kept", after=n0)
        return [(pm.copy(), rec)]
    if force == "box":
        new, err = box_try(pm, ext * 1e9)
        rec.update(method="box", err_mm=round(err, 2), after=12)
        return [(new, rec)]
    if force is None and depth == 0 and len({p.material_index for p in pm.polygons}) > 1 and ext.max() >= 10:
        # welded assembly of several colours (lens rings, connector shell + pins): rebuild each colour part on its own
        out = []
        for j, sp in enumerate(material_split(pm)):
            out += simplify_piece(sp, f"{key}.{j}", depth + 1)
            bpy.data.meshes.remove(sp)
        return out
    if force == "sections":
        out = []
        for j, sp in enumerate(section_split(pm)):
            out += simplify_piece(sp, f"{key}.s{j}", depth + 1)
            bpy.data.meshes.remove(sp)
        return out
    if force in (None, "lathe", "lathe-open"):
        ax = round_axis(co, tri)
        if ax[0] < 0.05 or force in ("lathe", "lathe-open"):
            LATHE_CANDS.append({"key": key, "me": pm.copy(), "d": ax[1], "c": ax[2], "score": ax[0], "force": force, "rec": rec})
            return []
    if force in (None, "plate") and sorted(ext)[0] < 0.2 * sorted(ext)[2] and ext.max() > 15:
        info = build.analyse_stepped(pm)
        if info:
            holes = build.plan_holes(info, min_diam_mm=2.0)
            drop = {(kk, idx) for (kk, idx, cc, req, Lp, match) in holes if req * 2 < 2.0e-3}
            cand = build.build_stepped(pm.name + " [plate]", info, drop)
            e1, e2 = build.fit_error(pm, cand, 3000)
            if e1 < 0.6 and e2 < 1.5 and build.open_edges(cand) == 0 and tri_count(cand) < n0:
                build.transfer_materials(pm, cand)
                rec.update(method="plate", err_mm=[round(e1, 2), round(e2, 2)], after=tri_count(cand))
                return [(cand, rec)]
            bpy.data.meshes.remove(cand)
    if force is None and (ext.max() < 15 or key.rsplit(".", 1)[-1].startswith("s")):  # small parts, and the box half of a sectioned piece
        new, err = box_try(pm, ext)
        if new is not None:
            rec.update(method="box", err_mm=round(err, 2), after=12)
            return [(new, rec)]
    if force in (None, "split") and depth == 0 and len({p.material_index for p in pm.polygons}) > 1:
        out = []
        for j, sp in enumerate(material_split(pm)):
            out += simplify_piece(sp, f"{key}.{j}", depth + 1)
            bpy.data.meshes.remove(sp)
        return out
    if force == "prep":  # plug small holes (< 4 mm) + planar dissolve, then a tight Collapse (0.3 mm)
        work = pm.copy()
        build.prep_for_decimate(work, plug_max=(LENS.get("plug_mm", 4) if LENS.get("case") == key else 4) * 1e-3)
        new, r, err = collapse(work, 0.3)
        bpy.data.meshes.remove(work)
        rec.update(method="plug + dissolve + collapse", ratio=round(r, 3), err_mm=round(err, 2), after=tri_count(new))
        return [(new, rec)]
    if force == "plugdissolve":  # plug small holes (< 6 mm) + Limited Dissolve, no Collapse (keeps the CAD winding)
        new = pm.copy()
        build.prep_for_decimate(new, plug_max=6e-3)
        bm = bmesh.new(); bm.from_mesh(new)
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
        bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(5), use_dissolve_boundaries=False, verts=bm.verts, edges=bm.edges,
                                 delimit={'MATERIAL'})
        bm.to_mesh(new); bm.free()
        rec.update(method="plug + dissolve", after=tri_count(new))
        return [(new, rec)]
    if force == "dissolve":  # shape-exact: Merge by Distance + Limited Dissolve only (flat-faced printed parts)
        new = pm.copy()
        bm = bmesh.new(); bm.from_mesh(new)
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
        bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(5), use_dissolve_boundaries=False, verts=bm.verts, edges=bm.edges,
                                 delimit={'MATERIAL'})
        bm.to_mesh(new); bm.free()
        rec.update(method="dissolve", after=tri_count(new))
        return [(new, rec)]
    new, r, err = collapse(pm, 0.2 + 0.01 * ext.max(), RATIO.get(key))
    rec.update(method="collapse", ratio=round(r, 3), err_mm=round(err, 2), after=tri_count(new))
    return [(new, rec)]


def to_library(me, key):
    """CAD colours -> RobotMaterials finishes on one part (overrides: '<piece key>:<cad material>' or '<cad material>')."""
    root = key.split(".")[0]
    slots, remap = [], []
    for m in me.materials:
        o_ = OVR.get(f"{key}:{m.name}", OVR.get(f"{root}:{m.name}", OVR.get(m.name)))
        L = m if o_ == "CAD" else LIBM[o_] if o_ else lib_for(m)
        MAPPING.setdefault(f"{root}:{m.name}", L.name)
        if L not in slots:
            slots.append(L)
        remap.append(slots.index(L))
    mi = np.empty(len(me.polygons), np.int32); me.polygons.foreach_get("material_index", mi)
    me.materials.clear()
    for L in slots:
        me.materials.append(L)
    if remap:
        me.polygons.foreach_set("material_index", np.array(remap, np.int32)[np.clip(mi, 0, len(remap) - 1)])


def orient(me):
    """Outward winding per connected piece, then flip closed shells with negative volume: Collapse and joins leave
    pieces inside-out, which Unity's backface culling shows as holes."""
    bm = bmesh.new(); bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me); bm.free(); me.update()
    build.fix_inverted_shells(me)


MAPPING = {}
for k, f in enumerate(isl):
    pm = explode.piece_mesh(A, f, f"{NAME}~{k:03d}")
    for new, rec in simplify_piece(pm, str(k)):
        log.append(rec)
        if new is not None:
            to_library(new, rec["k"])
            orient(new)
            parts.append(new)
            PARTS[rec["k"]] = new
    bpy.data.meshes.remove(pm)

groups = []
for cand in sorted(LATHE_CANDS, key=lambda x: -tri_count(x["me"])):
    for g in groups:
        g0 = g[0]
        dd = abs(float(np.dot(g0["d"], cand["d"])))
        off = np.asarray(cand["c"]) - np.asarray(g0["c"])
        perp = np.linalg.norm(off - np.dot(off, g0["d"]) * np.asarray(g0["d"]))
        if UNION and dd > 0.999 and perp < 0.6e-3 and not EXCL & {cand["key"], g0["key"]}:
            g.append(cand); break
    else:
        groups.append([cand])
for g in groups:
    built = None
    if len(g) > 1:
        # colours first (per piece), then one lathe around the union: rings take the nearest piece's finish
        for x in g:
            to_library(x["me"], x["key"])
        U = libcombo.join_meshes(NAME + " [union]", [(x["me"], Matrix.Identity(4)) for x in g])
        built, err = lathe_build(U, g[0]["d"], g[0]["c"], "lathe", union=True)
        if built is not None:
            parts.append(built)
            log.append({"k": "+".join(x["key"] for x in g), "method": "lathe (coaxial union)", "tris": sum(x["rec"]["tris"] for x in g),
                        "after": tri_count(built), "err_mm": round(err, 2)})
        bpy.data.meshes.remove(U)
    if built is None:
        for x in g:
            new, err = lathe_build(x["me"], x["d"], x["c"], x["force"])
            rec = x["rec"]
            if new is None:  # not a good lathe after all
                new, r, err = collapse(x["me"], 0.2 + 0.01 * max(rec["ext"]))
                rec.update(method="collapse", ratio=round(r, 3))
            else:
                rec.update(method="lathe", round_score=round(x["score"], 4))
            rec.update(after=tri_count(new), err_mm=round(err, 2))
            log.append(rec)
            to_library(new, rec["k"]) if new.materials and new.materials[0].name not in LIBM else None
            orient(new)
            parts.append(new)
            PARTS[rec["k"]] = new
    for x in g:
        bpy.data.meshes.remove(x["me"])
if LENS:
    lens_keys = [k for k in LENS["lens"] if k in PARTS]
    ax = next(x for x in LATHE_CANDS if x["key"] in LENS["lens"])
    d, c = np.asarray(ax["d"], float), np.asarray(ax["c"], float)
    case_me = PARTS[LENS["case"]]
    cco_, _, _ = geo.mesh_arrays(case_me)
    rel = cco_ - c; h = rel @ d; r = np.linalg.norm(rel - np.outer(h, d), axis=1)
    lco_ = np.vstack([geo.mesh_arrays(PARTS[k])[0] for k in lens_keys])
    lh = (lco_ - c) @ d
    rmax = float(np.linalg.norm(lco_ - c - np.outer(lh, d), axis=1).max())
    near = r <= max(rmax + 4e-3, float(r.min()) + 1.5e-3)  # the plugged opening's rim (closest case vertices)
    gk = LENS.get("glass")  # the front element (glass) decides which way is out
    gh = float(((geo.mesh_arrays(PARTS[gk])[0] - c) @ d).mean()) if gk in PARTS else float(lh.mean())
    front = 1.0 if gh > float(np.median(h[near])) else -1.0
    h_case = float((front * h[near]).max() * front)  # the case's outermost face around the lens
    for k in lens_keys:
        bm = bmesh.new(); bm.from_mesh(PARTS[k])
        bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], plane_co=Vector(c + d * h_case),
                               plane_no=Vector(d * front), clear_inner=True)
        bm.to_mesh(PARTS[k]); bm.free(); PARTS[k].update()
        log.append({"k": k, "method": "lens cut at the case face", "after": tri_count(PARTS[k])})
    parts = [m for m in parts if len(m.polygons)]
    print("CAMASSET lens cut at", round(h_case / MM, 2), "mm along the lens axis, front", front, flush=True)
me = libcombo.join_meshes(NAME, [(m, Matrix.Identity(4)) for m in parts])
for m in parts:
    bpy.data.meshes.remove(m)
mapping = MAPPING
# final Limited Dissolve per mesh (owner's step), no normals delimit across finishes
bm = bmesh.new(); bm.from_mesh(me)
bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(1.0), use_dissolve_boundaries=False, verts=bm.verts, edges=bm.edges,
                         delimit={'MATERIAL'})
bm.to_mesh(me); bm.free()
build.fix_inverted_shells(me)
me.update()
me.name = NAME


def prune_hidden_faces(me, nviews=96, res=900):
    """Delete faces that never show up in flat face-id renders from `nviews` directions with backface culling on
    (owner: model only what can be seen; Unity culls back faces). Deleting faces nobody can see changes none of
    these renders, so no see-through gaps appear -- unlike a sparse ray test, which missed faces seen only through
    thin slits (between a case and its back plate, between lens rings)."""
    sc = bpy.context.scene
    tmp = bpy.data.objects.new("__prune", me); sc.collection.objects.link(tmp)
    hidden_state = {x.name: x.hide_render for x in sc.objects}
    for x in sc.objects:
        x.hide_render = x is not tmp
    nf = len(me.polygons)
    ids = np.repeat(np.arange(1, nf + 1), [len(p.vertices) for p in me.polygons])
    col = np.zeros((len(ids), 4), np.float32)
    col[:, 0] = ((ids & 31) * 8 + 4) / 255; col[:, 1] = (((ids >> 5) & 31) * 8 + 4) / 255
    col[:, 2] = (((ids >> 10) & 31) * 8 + 4) / 255; col[:, 3] = 1
    attr = me.color_attributes.new("__fid", 'FLOAT_COLOR', 'CORNER')
    attr.data.foreach_set("color", col.ravel())
    me.color_attributes.active_color = attr
    r = sc.render; sh = sc.display.shading
    keep_r = (r.engine, r.resolution_x, r.resolution_y, r.film_transparent, r.dither_intensity, sc.display.render_aa,
              sh.light, sh.color_type, sh.show_backface_culling, sh.show_cavity, sh.show_object_outline, sh.show_specular_highlight,
              sc.view_settings.view_transform, sc.camera, r.filepath)
    r.engine = 'BLENDER_WORKBENCH'; r.resolution_x = r.resolution_y = res; r.film_transparent = True; r.dither_intensity = 0
    sc.display.render_aa = 'OFF'; sh.light = 'FLAT'; sh.color_type = 'VERTEX'; sh.show_backface_culling = True
    sh.show_cavity = sh.show_object_outline = sh.show_specular_highlight = False; sc.view_settings.view_transform = 'Raw'
    camd = bpy.data.cameras.new("__pc"); cam = bpy.data.objects.new("__pc", camd); sc.collection.objects.link(cam); sc.camera = cam
    camd.type = 'ORTHO'
    co_, _, _ = geo.mesh_arrays(me); ctr = Vector(0.5 * (co_.min(0) + co_.max(0))); R = float(np.linalg.norm(np.ptp(co_, 0))) / 2
    camd.ortho_scale = 2.04 * R; camd.clip_start = 1e-4; camd.clip_end = 8 * R
    seen = np.zeros(nf + 1, bool)
    path = os.path.join(WD, "__prune.png")
    k = np.arange(nviews) + 0.5; z = 1 - 2 * k / nviews; th = k * math.pi * (3 - math.sqrt(5))
    dirs = [Vector((math.sqrt(1 - zz * zz) * math.cos(t), math.sqrt(1 - zz * zz) * math.sin(t), zz)) for zz, t in zip(z, th)]
    dirs += [Vector(v) for v in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))]
    for d in dirs:
        cam.location = ctr + d * 3 * R
        cam.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
        r.filepath = path
        bpy.ops.render.render(write_still=True)
        img = bpy.data.images.load(path); img.colorspace_settings.name = 'Non-Color'
        px = np.empty(res * res * 4, np.float32); img.pixels.foreach_get(px); bpy.data.images.remove(img)
        px = px.reshape(-1, 4); m = px[:, 3] > 0.5
        c = np.round(px[m, :3] * 255).astype(np.int64)
        fid = (c[:, 0] // 8) | ((c[:, 1] // 8) << 5) | ((c[:, 2] // 8) << 10)
        seen[fid[fid <= nf]] = True
    me.color_attributes.remove(me.color_attributes["__fid"])
    (r.engine, r.resolution_x, r.resolution_y, r.film_transparent, r.dither_intensity, sc.display.render_aa,
     sh.light, sh.color_type, sh.show_backface_culling, sh.show_cavity, sh.show_object_outline, sh.show_specular_highlight,
     sc.view_settings.view_transform, sc.camera, r.filepath) = keep_r
    bpy.data.objects.remove(cam); bpy.data.cameras.remove(camd); bpy.data.objects.remove(tmp)
    for x in sc.objects:
        if x.name in hidden_state:
            x.hide_render = hidden_state[x.name]
    bm = bmesh.new(); bm.from_mesh(me); bm.faces.ensure_lookup_table()
    dead = [f for f in bm.faces if not seen[f.index + 1]]
    bmesh.ops.delete(bm, geom=dead, context='FACES')
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context='VERTS')
    bm.to_mesh(me); bm.free(); me.update()
    return len(dead)


t_before = tri_count(me)
n_pruned = prune_hidden_faces(me)
me.shade_smooth()
me.set_sharp_from_angle(angle=math.radians(30))  # flat faces crisp, round parts smooth
me.update()
print("CAMASSET pruned hidden faces", n_pruned, "tris", t_before, "->", tri_count(me), flush=True)

# fit check against the CAD (both centred the same way)
cco, ctri, _ = geo.mesh_arrays(src)
aco, atri, _ = geo.mesh_arrays(me)
P, _ = geo.sample_surface(aco, atri, 5000); Q, _ = geo.sample_surface(cco, ctri, 20000)
ch = align.chamfer(P, Q, np.eye(3), np.zeros(3))
tris = tri_count(me)
summary = {"asset": NAME, "cad": CAD, "cad_tris": tri_count(cad.data), "tris": tris, "chamfer_mm": [round(c * 1000, 2) for c in ch],
           "centre_offset_m": centre.tolist(), "materials": mapping, "pieces": log}
json.dump(summary, open(os.path.join(WD, NAME.split(" (")[0].replace("/", "-") + "_asset.json"), "w"), indent=1)
print("CAMASSET", NAME, "tris", tris, "from", summary["cad_tris"], "chamfer mm", summary["chamfer_mm"],
      "pieces", {m: sum(1 for r in log if r.get("method") == m) for m in {r.get("method") for r in log}}, flush=True)

# preview + before/after renders (Workbench, cavity off) from 3 directions
sc = bpy.context.scene
ob = bpy.data.objects.new(NAME, me); sc.collection.objects.link(ob)
ref_me = src
ro = bpy.data.objects.new(NAME + " (CAD)", ref_me); sc.collection.objects.link(ro)
sys.path.insert(0, os.path.join(HERE, "cleanup"))
import clean_core as cc
cc.setup_render(600)
sc.display.shading.show_backface_culling = True
for x in sc.objects:
    if x.type == 'MESH' and x not in (ob, ro):
        x.hide_render = True
for i, dvec in enumerate([(1, -1.3, 0.9), (-1, 1.3, 0.9), (0.3, -1, -0.6)]):
    cc.render_obj(ro, os.path.join(WD, f"{NAME[:20]}_{i}_cad.png"), dvec)
    cc.render_obj(ob, os.path.join(WD, f"{NAME[:20]}_{i}_asset.png"), dvec)
PREVIEW = os.path.join(WD, f"{NAME[:20]}_0_asset.png")

# new file content: only the asset + the CAD reference
for o in list(bpy.data.objects):
    if o not in (ob, ro):
        bpy.data.objects.remove(o)
for c in list(bpy.data.collections):
    bpy.data.collections.remove(c)
sc.collection.objects.unlink(ro)
ref = bpy.data.collections.new("CAD reference")
sc.collection.children.link(ref)
ref.objects.link(ro)
bpy.context.view_layer.layer_collection.children["CAD reference"].exclude = True
ob.asset_mark()
ob.asset_data.catalog_id = CATALOG
ob.asset_data.description = (f"Low-poly {NAME}, built from {os.environ.get('CAM_ROBOT', 'a robot')}'s CAD "
                             f"({summary['cad_tris']:,} tris -> {tris:,}).")  # CAM_ROBOT: robot the CAD came from
for t in ("camera", "vision", "electronics"):
    ob.asset_data.tags.new(t)
try:
    with bpy.context.temp_override(id=ob):
        bpy.ops.ed.lib_id_load_custom_preview(filepath=PREVIEW)
except Exception as ex:
    print("CAMASSET preview failed", ex, flush=True)
sc.camera = None
for c in list(bpy.data.cameras):
    bpy.data.cameras.remove(c)
bpy.ops.outliner.orphans_purge(do_recursive=True)
bpy.context.preferences.filepaths.save_version = 0
if os.path.exists(OUT):
    raise SystemExit(f"refusing to overwrite {OUT}")
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
print("CAMASSET saved", OUT, flush=True)
