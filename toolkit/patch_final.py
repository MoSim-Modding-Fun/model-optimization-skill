# Targeted fixes on a finished robot file, after QA (no full pipeline rerun):
#  - live Collapse ratios retuned per object (heavy flat parts lower, torn parts higher)
#  - parts rebuilt as one hex prism per CAD piece (a shaft exported as two stubs must not become one bridging bar)
#   ROBOT=<robot> blender -b in.blend --python patch_final.py -- out.blend patch.json <cleaned CAD blend>
import bpy, sys, os, json
import numpy as np
from mathutils import Matrix
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build, explode, libcombo
OUT, SPEC, CAD = sys.argv[sys.argv.index("--") + 1:][:3]
spec = json.load(open(SPEC))
bpy.context.preferences.filepaths.save_version = 0
log = {}
for on, ratio in spec.get("collapse_ratio", {}).items():
    o = bpy.data.objects[on]
    m = next(m for m in o.modifiers if m.type == 'DECIMATE' and m.decimate_type == 'COLLAPSE')
    log[on] = {"ratio": [round(m.ratio, 3), ratio]}
    m.ratio = ratio
names = spec.get("hex_per_piece", [])
if names:
    with bpy.data.libraries.load(CAD, link=False) as (src, dst):
        dst.objects = list(names)  # Blender swaps the list entries for the loaded objects
    for name, cad in zip(names, dst.objects):  # the appended CAD object gets a .001 name; `name` stays the robot's object
        target = bpy.data.objects[name]
        me = cad.data
        A = explode.MeshArrays(me)
        parts = []
        for f in explode.islands(me):
            pm = explode.piece_mesh(A, f, "piece")
            hx, dev, ap = build.hex_candidate(pm)
            if hx is None:
                parts.append(pm); continue
            build.transfer_materials(pm, hx)
            parts.append(hx)
            bpy.data.meshes.remove(pm)
        new = libcombo.join_meshes(name + " [hex x%d]" % len(parts), [(q, Matrix.Identity(4)) for q in parts])
        for q in parts:
            bpy.data.meshes.remove(q)
        old = target.data
        for x in [x for x in bpy.data.objects if x.data is old and x is not cad]:
            x.data = new
        log[name] = {"hex_per_piece": len(parts), "tris": sum(len(q.vertices) - 2 for q in new.polygons)}
        bpy.data.objects.remove(cad)
# plates whose stepped rebuild kept tiny holes: plug holes < plug_mm first, then the stepped rebuild (accepted only when
# closed and within 0.5 mm of the CAD)
pst = spec.get("plug_stepped_from_cad", {})
if pst:
    import bmesh
    with bpy.data.libraries.load(CAD, link=False) as (src, dst):
        dst.objects = list(pst)
    for (name, plug_mm), cad in zip(list(pst.items()), dst.objects):
        target = bpy.data.objects[name]
        me = cad.data.copy()
        if "custom_normal" in me.attributes:
            me.attributes.remove(me.attributes["custom_normal"])
        bm = bmesh.new(); bm.from_mesh(me); bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5); bm.to_mesh(me); bm.free()
        src_me = me.copy()
        k, _ = build.prep_for_decimate(me, plug_max=plug_mm * 1e-3)
        info = build.analyse_stepped(me)
        new = None
        if info:
            holes = build.plan_holes(info)
            drop = {(kk, idx) for (kk, idx, cc, req, Lp, match) in holes if req * 2 < plug_mm * 1e-3}
            new = build.build_stepped(name + " [plate]", info, drop)
            e1, e2 = build.fit_error(src_me, new, 4000)
            if build.open_edges(new) or max(e1, e2) > 0.5:
                bpy.data.meshes.remove(new); new = None
        if new is None:
            log[name] = {"plug_stepped_from_cad": "rejected, kept as is"}
        else:
            build.transfer_materials(src_me, new)
            old = target.data
            for x in [x for x in bpy.data.objects if x.data is old and x is not cad]:
                x.modifiers.clear()
                x.data = new
            log[name] = {"plug_stepped_from_cad": sum(len(q.vertices) - 2 for q in new.polygons), "holes_plugged": k,
                         "err_mm": [round(e1, 2), round(e2, 2)]}
        bpy.data.objects.remove(cad)
# flat plates of one thickness the stepped rebuild rejects: plug small holes, take the top face's boundary loops,
# simplify them (0.3 mm) and extrude between the two faces (outline and slots kept, nothing else)
ext = spec.get("extrude_from_cad", {})
if ext:
    import geo, shading, collections as _c
    from mathutils import Vector
    with bpy.data.libraries.load(CAD, link=False) as (src, dst):
        dst.objects = list(ext)
    for (name, plug_mm), cad in zip(list(ext.items()), dst.objects):
        target = bpy.data.objects[name]
        me = cad.data.copy()
        if "custom_normal" in me.attributes:
            me.attributes.remove(me.attributes["custom_normal"])
        import bmesh
        bm = bmesh.new(); bm.from_mesh(me); bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5); bm.to_mesh(me); bm.free()
        src_me = me.copy()
        build.prep_for_decimate(me, plug_max=plug_mm * 1e-3)
        co = np.array([v.co for v in me.vertices])
        ax = int(np.argmin(np.ptp(co, 0))); d = np.zeros(3); d[ax] = 1.0
        u, w = geo.perp_basis(d)
        h = co @ d; h0, h1 = float(h.min()), float(h.max())
        from mathutils.bvhtree import BVHTree
        bvh = BVHTree.FromPolygons([v.co for v in me.vertices], [tuple(p_.vertices) for p_ in me.polygons])

        def face_loops(sign, level):
            sel = [p for p in me.polygons if p.normal.dot(d) * sign > 0.99 and abs(float(np.dot(p.center, d)) - level) < 5e-4]
            ecount = _c.Counter()
            for p in sel:
                for ek in p.edge_keys:
                    ecount[ek] += 1
            adj = _c.defaultdict(list)
            for (a_, b_), n in ecount.items():
                if n == 1:
                    adj[a_].append(b_); adj[b_].append(a_)
            seen, out = set(), []
            for st in list(adj):
                if st in seen:
                    continue
                loop, prev, cur = [st], None, st
                seen.add(st)
                while True:
                    nxt = [x for x in adj[cur] if x != prev and x not in seen]
                    if not nxt:
                        break
                    prev, cur = cur, nxt[0]; seen.add(cur); loop.append(cur)
                if len(loop) >= 3:
                    P = np.asarray(geo.douglas_peucker(np.array([[co[i] @ u, co[i] @ w] for i in loop]), 3e-4, closed=True))
                    if len(P) >= 3 and abs(geo.poly_area(P)) > (plug_mm * 1e-3) ** 2 * 0.5:
                        out.append(P)
            return out

        def through_share(P):
            V2, F2 = build.triangulate_region([P])
            if not len(F2):
                return 0.0
            free = 0
            for f in F2:
                c2 = np.mean([V2[i] for i in f], axis=0)
                hit = bvh.ray_cast(Vector(u * c2[0] + w * c2[1] + d * (h1 + 1e-3)), Vector(-d), (h1 - h0) + 2e-3)
                free += hit[0] is None
            return free / len(F2)

        tops, bots = face_loops(1, h1), face_loops(-1, h0)
        outer = max(tops, key=lambda L_: abs(geo.poly_area(L_)))
        cands = [L_ for L_ in tops if L_ is not outer] + [L_ for L_ in bots if abs(geo.poly_area(L_)) < 0.9 * abs(geo.poly_area(outer))]
        through = [L_ for L_ in cands if through_share(L_) > 0.6]
        through.sort(key=lambda L_: abs(geo.poly_area(L_)))  # narrowest first: a groove's top loop loses to the slit under it
        loops = [outer]
        for L_ in through:
            c2 = L_.mean(0)
            if not any(np.linalg.norm(c2 - K.mean(0)) < 5e-3 for K in loops[1:]):
                loops.append(L_)
        new = build.extrude_loops(name + " [extruded]", loops, u, w, d, h0, h1)
        build.transfer_materials(src_me, new)
        e1, e2 = build.fit_error(src_me, new, 4000)
        old = target.data
        for x in [x for x in bpy.data.objects if x.data is old and x is not cad]:
            x.modifiers.clear()
            x.data = new
        log[name] = {"extrude_from_cad": sum(len(q.vertices) - 2 for q in new.polygons), "loops": len(loops),
                     "err_mm": [round(e1, 2), round(e2, 2)], "open_edges": build.open_edges(new)}
        bpy.data.objects.remove(cad)
# flat plates: CAD with small holes plugged (< plug_mm) + Limited Dissolve, no Collapse (outline and slots exact)
plug = spec.get("plug_dissolve_from_cad", {})
if plug:
    import bmesh, math, shading
    with bpy.data.libraries.load(CAD, link=False) as (src, dst):
        dst.objects = list(plug)
    for (name, plug_mm), cad in zip(list(plug.items()), dst.objects):
        target = bpy.data.objects[name]
        new = cad.data.copy(); new.name = name + " [plug+dissolve]"
        if "custom_normal" in new.attributes:
            new.attributes.remove(new.attributes["custom_normal"])
        bm = bmesh.new(); bm.from_mesh(new)
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
        bm.to_mesh(new); bm.free()
        n_plugged, _ = build.prep_for_decimate(new, plug_max=plug_mm * 1e-3)
        bm = bmesh.new(); bm.from_mesh(new)
        bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(5), use_dissolve_boundaries=False, verts=bm.verts, edges=bm.edges,
                                 delimit={'MATERIAL'})
        bm.to_mesh(new); bm.free()
        build.fix_inverted_shells(new)
        old = target.data
        for x in [x for x in bpy.data.objects if x.data is old and x is not cad]:
            x.modifiers.clear()
            x.data = new
            shading.add_shading(x)
        log[name] = {"plug_dissolve_from_cad": sum(len(q.vertices) - 2 for q in new.polygons), "holes_plugged": n_plugged}
        bpy.data.objects.remove(cad)
# CAD shape kept exactly (Merge by Distance + Limited Dissolve), for small parts a rebuild got wrong
names = spec.get("dissolve_from_cad", [])
if names:
    import bmesh, math, shading
    with bpy.data.libraries.load(CAD, link=False) as (src, dst):
        dst.objects = list(names)
    for name, cad in zip(names, dst.objects):
        target = bpy.data.objects[name]
        new = cad.data.copy(); new.name = name + " [dissolve]"
        if "custom_normal" in new.attributes:
            new.attributes.remove(new.attributes["custom_normal"])
        bm = bmesh.new(); bm.from_mesh(new)
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
        bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(5), use_dissolve_boundaries=False, verts=bm.verts, edges=bm.edges,
                                 delimit={'MATERIAL'})
        bm.to_mesh(new); bm.free()
        old = target.data
        for x in [x for x in bpy.data.objects if x.data is old and x is not cad]:
            x.modifiers.clear()
            x.data = new
            shading.add_shading(x)
        log[name] = {"dissolve_from_cad": sum(len(q.vertices) - 2 for q in new.polygons)}
        bpy.data.objects.remove(cad)
json.dump(log, open(os.path.splitext(OUT)[0] + "_patch.json", "w"), indent=1)
print("PATCH", json.dumps(log), flush=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
