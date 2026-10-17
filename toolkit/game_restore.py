# Make a simplified robot a drop-in replacement for the game's own FBX (2026-10-15). A Unity prefab that has the
# FBX unpacked keeps its own GameObjects, transforms and per-submesh materials; it only points at the FBX's meshes (one
# internal fileID per FBX geometry). So after the pipeline every object must still use a mesh with its ORIGINAL datablock
# name, objects that shared a mesh must still share one, and each mesh must keep its original material slot list (count
# and order) with at least one face per slot (the renderer's material i goes on submesh i).
#   1. dump the original:  blender -b orig.blend --python game_restore.py -- dump orig_map.json
#   2. restore:            blender -b final.blend --python game_restore.py -- restore orig_map.json orig.blend out.blend report.json [not_in_game.json]
#      (optional: object names the game prefab doesn't use -> collection "REVIEW - Not in game", for the owner)
# What restore does, per original mesh name M:
#   - the objects that used M must now share ONE mesh X (else: error in the report, the first object's mesh wins);
#     X is copied if objects of another original mesh share it too, then renamed to M (an old datablock named M is renamed
#     out of the way first);
#   - X's slots become M's original list: a face whose material is in that list keeps it (first slot with that material);
#     a face with any other material (a library asset's own finish) takes the slot of the nearest original face; a slot
#     left without faces gets a "slot keeper": one 0.05 mm triangle 0.1 mm under the surface at an original face of that
#     slot (invisible, keeps the submesh and so the material order in Unity);
#   - objects the run added (Kraken caps: merged into the shared motor mesh when every user of that mesh has the same cap;
#     exploded REVIEW pieces) are reported; parents and local matrices are compared with the original.
import bpy, bmesh, sys, os, json, math
import numpy as np
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

a = sys.argv[sys.argv.index("--") + 1:]
MODE = a[0]


LIB_DE_MAX = float(os.environ.get("LIB_DE_MAX", "20"))


def mat_lab(m):
    """CIELAB of a material's base colour (Principled Base Color, else the viewport colour), linear -> sRGB D65."""
    c = None
    if m.use_nodes and m.node_tree:
        for nd in m.node_tree.nodes:
            if nd.bl_idname == 'ShaderNodeBsdfPrincipled' and not nd.inputs['Base Color'].is_linked:
                c = nd.inputs['Base Color'].default_value[:3]
                break
    if c is None:
        c = m.diffuse_color[:3]
    xyz = [sum(k * v for k, v in zip(row, c)) for row in ((0.4124, 0.3576, 0.1805), (0.2126, 0.7152, 0.0722), (0.0193, 0.1192, 0.9505))]
    wp = (0.95047, 1.0, 1.08883)
    f = [(t / w) ** (1 / 3) if t / w > 0.008856 else 7.787 * t / w + 16 / 116 for t, w in zip(xyz, wp)]
    return (116 * f[1] - 16, 500 * (f[0] - f[1]), 200 * (f[1] - f[2]))


def de(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def mat_names(me):
    return [m.name if m else None for m in me.materials]


if MODE == "dump":
    out = {"objects": {}, "meshes": {}}
    for o in bpy.data.objects:
        out["objects"][o.name] = {"type": o.type, "mesh": o.data.name if o.type == 'MESH' else None,
                                  "parent": o.parent.name if o.parent else None,
                                  "basis": [list(r) for r in o.matrix_basis]}
    for me in bpy.data.meshes:
        if me.users:
            used = sorted({p.material_index for p in me.polygons})
            out["meshes"][me.name] = {"mats": mat_names(me), "used": used, "faces": len(me.polygons)}
    json.dump(out, open(a[1], "w"))
    print("DUMP", len(out["objects"]), "objects", len(out["meshes"]), "meshes")
    raise SystemExit(0)

ORIG = json.load(open(a[1]))
ORIG_BLEND, OUT, REPORT = a[2], a[3], a[4]
NOT_IN_GAME = json.load(open(a[5])) if len(a) > 5 else []
rep = {"errors": [], "split_meshes": [], "copied": [], "renamed": 0, "remapped_faces": {}, "slot_keepers": {},
       "added_objects": [], "merged_caps": [], "missing_objects": [], "transform_diffs": [], "parent_diffs": []}
sc = bpy.context.scene
objs = bpy.data.objects

# ------------------------------------------------------------------ objects: missing / added / parents / matrices
for n, info in ORIG["objects"].items():
    o = objs.get(n)
    if o is None:
        rep["missing_objects"].append(n)
        continue
    if (o.parent.name if o.parent else None) != info["parent"]:
        rep["parent_diffs"].append([n, info["parent"], o.parent.name if o.parent else None])
    d = max(abs(x - y) for r1, r2 in zip(o.matrix_basis, info["basis"]) for x, y in zip(r1, r2))
    if d > 1e-6:
        rep["transform_diffs"].append([n, d])
added = [o for o in objs if o.name not in ORIG["objects"]]

# ------------------------------------------------------------------ Kraken caps -> into the shared motor mesh
# pipeline.py adds a "kraken cap" child per motor whose CAD had a cap (owner rule). A new object would never reach the
# game (the prefab doesn't know it), so the cap goes into the motor's mesh, which every user of that mesh shares: only
# when ALL users have a cap at the same local matrix.
caps = [o for o in added if o.type == 'MESH' and o.parent is not None and o.name.lower().startswith("kraken cap")]
by_motor_mesh = {}
for c in caps:
    by_motor_mesh.setdefault(c.parent.data.name, []).append(c)
for mname, cs in by_motor_mesh.items():
    users = [o for o in objs if o.type == 'MESH' and o.data.name == mname and o.name in ORIG["objects"]]
    mats = [c.matrix_basis for c in cs]
    same = all(max(abs(x - y) for r1, r2 in zip(m, mats[0]) for x, y in zip(r1, r2)) < 1e-6 for m in mats)
    if {c.parent.name for c in cs} == {u.name for u in users} and same:
        me = bpy.data.meshes[mname]
        cap = cs[0].data.copy()
        cap.transform(mats[0])
        bm = bmesh.new()
        bm.from_mesh(me)
        nf = len(bm.faces)
        # cap faces keep their own material by name (remapped onto the motor's slots below)
        idx = []
        for m in cap.materials:
            if m not in list(me.materials):
                me.materials.append(m)
            idx.append(list(me.materials).index(m))
        bm2 = bmesh.new()
        bm2.from_mesh(cap)
        for f in bm2.faces:
            f.material_index = idx[f.material_index] if idx else 0
        tmp = bpy.data.meshes.new("__cap_tmp")
        bm2.to_mesh(tmp)
        bm2.free()
        bm.from_mesh(tmp)
        bm.to_mesh(me)
        bm.free()
        bpy.data.meshes.remove(tmp)
        bpy.data.meshes.remove(cap)
        for c in cs:
            rep["merged_caps"].append([c.name, c.parent.name])
            bpy.data.objects.remove(c)
    else:
        rep["errors"].append(f"kraken caps on only some users of {mname} ({len(cs)} caps, {len(users)} users): left as objects")
added = [o for o in objs if o.name not in ORIG["objects"]]
rep["added_objects"] = sorted(o.name for o in added)

# ------------------------------------------------------------------ mesh identity
users_of = {}
for n, info in ORIG["objects"].items():
    if info["mesh"] and n in objs:
        users_of.setdefault(info["mesh"], []).append(objs[n])
final_users = {}
for o in objs:
    if o.type == 'MESH':
        final_users.setdefault(o.data.name, set()).add(o.name)
plan = {}  # original mesh name -> final mesh datablock
for M, us in users_of.items():
    fm = {}
    for o in us:
        fm.setdefault(o.data.name, []).append(o.name)
    if len(fm) > 1:
        rep["split_meshes"].append([M, {k: len(v) for k, v in fm.items()}])
        # the fullest mesh wins, not the first user's: an exploded CAD duplicate whose pieces all went to REVIEW
        # keeps an empty mesh, and as the first user it emptied its visible twin (two MK5n turrets + a wheel, 2-4 tris)
        X = max(us, key=lambda o: len(o.data.polygons)).data
        for o in us:
            o.data = X
    plan[M] = us[0].data
claimed = {}
for M, X in plan.items():
    claimed.setdefault(X.name, []).append(M)
for xname, Ms in claimed.items():
    X = bpy.data.meshes[xname]
    for M in Ms[1:]:  # one datablock stands for several original meshes: each gets its own copy
        C = X.copy()
        for o in users_of[M]:
            o.data = C
        plan[M] = C
        rep["copied"].append([M, xname])
for M, X in plan.items():  # free every target name first (the pipeline kept the CAD datablocks around under them)
    old = bpy.data.meshes.get(M)
    if old is not None and old != X:
        old.name = M + " __old"
for M, X in plan.items():
    if X.name != M:
        X.name = M
        rep["renamed"] += 1
        if X.name != M:
            rep["errors"].append(f"could not rename to {M}: got {X.name}")

# ------------------------------------------------------------------ original geometry (only where it is needed)
need = []
for M, X in plan.items():
    want = ORIG["meshes"][M]["mats"]
    cur = mat_names(X)
    used = {p.material_index for p in X.polygons}
    if cur != want or any(cur[i] not in want for i in used if i < len(cur)) or len(used) < len(want):
        need.append(M)
orig_me = {}
if need:
    with bpy.data.libraries.load(ORIG_BLEND, link=False) as (src, dst):
        names = [m for m in need if m in src.meshes]
        dst.meshes = list(names)
    for M, me in zip(names, dst.meshes):  # appended under free names (".001"): map back by request order
        orig_me[M] = me
        me.name = "__orig__ " + M
    for M in need:
        if M not in orig_me:
            rep["errors"].append(f"original mesh {M} not in {ORIG_BLEND}")
    need = [M for M in need if M in orig_me]

mat_by_name = {m.name: m for m in bpy.data.materials}
missing_mats = sorted({n for M in plan for n in ORIG["meshes"][M]["mats"] if n and n not in mat_by_name})
if missing_mats:  # a material the run removed: take it back from the original file
    with bpy.data.libraries.load(ORIG_BLEND, link=False) as (src, dst):
        dst.materials = [m for m in src.materials if m in missing_mats]
    mat_by_name = {m.name: m for m in bpy.data.materials}
    rep["materials_restored"] = missing_mats


def orig_tris(me):
    me.calc_loop_triangles()
    co = np.array([v.co for v in me.vertices])
    tri = np.array([t.vertices for t in me.loop_triangles], np.int64)
    mi = np.array([me.polygons[t.polygon_index].material_index for t in me.loop_triangles], np.int64)
    nrm = np.array([t.normal for t in me.loop_triangles])
    return co, tri, mi, nrm


for M in need:
    X = plan[M]
    want = ORIG["meshes"][M]["mats"]
    cur = mat_names(X)
    O = orig_me.get(M)
    co, tri, omi, onrm = orig_tris(O)
    bvh = BVHTree.FromPolygons([Vector(c) for c in co], [tuple(t) for t in tri])
    first = {}
    for i, n in enumerate(want):
        first.setdefault(n, i)
    # a material that isn't in the original list (a library asset's finish) goes onto ONE original slot as a whole: the
    # slot whose original faces lie nearest to most of its area (as build.recolour_from_cad). Per-face nearest painted
    # library Krakens with stripes of the CAD's connector colours (run 1).
    votes = {}
    for p in X.polygons:
        n = cur[p.material_index] if p.material_index < len(cur) else None
        if n not in first:
            loc, nrm, fi, d = bvh.find_nearest(p.center)
            if fi is not None:
                v = votes.setdefault(n, {})
                v[int(omi[fi])] = v.get(int(omi[fi]), 0.0) + p.area
    slot_of = {n: max(v, key=v.get) for n, v in votes.items()}
    # colour first (CIELAB distance): a library finish goes onto the CAD slot of the nearest colour when one is close
    # (dE < LIB_DE_MAX), so the library's own colour regions survive where the CAD has a similar slot (red PDP breakers,
    # an amber RSL ring); otherwise the area vote above
    for n in list(slot_of):
        m = mat_by_name.get(n)
        if m is None:
            continue
        lab = mat_lab(m)
        best = min(((de(lab, mat_lab(mat_by_name[w])), i) for i, w in enumerate(want) if w in mat_by_name), default=None)
        if best is not None and best[0] < LIB_DE_MAX:
            slot_of[n] = best[1]
            rep.setdefault("colour_matched", {}).setdefault(M, []).append([n, want[best[1]], round(best[0], 1)])
    n_remap = 0
    new_idx = []
    for p in X.polygons:
        n = cur[p.material_index] if p.material_index < len(cur) else None
        if n in first:
            new_idx.append(first[n])
        else:
            new_idx.append(slot_of.get(n, 0))
            n_remap += 1
    if slot_of:
        rep.setdefault("material_map", {})[M] = {str(k): want[v] for k, v in slot_of.items()}
    X.materials.clear()
    for n in want:
        X.materials.append(mat_by_name.get(n) if n else None)
    X.polygons.foreach_set("material_index", new_idx)
    if n_remap:
        rep["remapped_faces"][M] = n_remap
    # slot keepers for slots without faces
    used = set(new_idx)
    lost = [s for s in range(len(want)) if s not in used]
    if lost:
        bm = bmesh.new()
        bm.from_mesh(X)
        Xbvh = BVHTree.FromBMesh(bm)
        for s in lost:
            cand = np.nonzero(omi == s)[0]
            if len(cand) == 0:
                cand = np.arange(len(tri))
            t = int(cand[len(cand) // 2])
            c = Vector(co[tri[t]].mean(axis=0))
            loc, nrm, fi, d = Xbvh.find_nearest(c)
            if loc is None:
                loc, nrm = c, Vector(onrm[t])
            n_ = nrm.normalized()
            u = n_.orthogonal().normalized()
            v = n_.cross(u)
            base = loc - n_ * 1e-4
            r = 2.5e-5
            vs = [bm.verts.new(base + (u * math.cos(k * 2.094) + v * math.sin(k * 2.094)) * r) for k in range(3)]
            f = bm.faces.new(vs)
            f.material_index = s
            f.normal_update()
            if f.normal.dot(n_) < 0:
                f.normal_flip()
            rep["slot_keepers"].setdefault(M, []).append([s, want[s]])
        bm.to_mesh(X)
        bm.free()
    X.update()
for me in orig_me.values():
    bpy.data.meshes.remove(me)

# ------------------------------------------------------------------ one shading stack per shared mesh
# users of one mesh must carry the same live stack (a CAD duplicate sent to REVIEW gets none from the pipeline):
# the ones without Smooth by Angle -> WeightedNormal get it, so the .blend and its export agree
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shading
rep["shading_synced"] = []
for M, X in plan.items():
    us = [o for o in objs if o.type == 'MESH' and o.data == X]
    has = [any(m.type == 'WEIGHTED_NORMAL' for m in o.modifiers) for o in us]
    if any(has) and not all(has):
        for o, h in zip(us, has):
            if not h:
                shading.add_shading(o)
                rep["shading_synced"].append(o.name)

# ------------------------------------------------------------------ objects the game prefab doesn't use
if NOT_IN_GAME:
    nig = bpy.data.collections.get("REVIEW - Not in game") or bpy.data.collections.new("REVIEW - Not in game")
    if nig.name not in sc.collection.children:
        sc.collection.children.link(nig)
    nig.color_tag = 'COLOR_07'
    moved = []
    for n in NOT_IN_GAME:
        o = objs.get(n)
        if o is None:
            continue
        for c in list(o.users_collection):
            c.objects.unlink(o)
        nig.objects.link(o)
        moved.append(n)
    rep["not_in_game"] = moved

# ------------------------------------------------------------------ final check
bad = []
for M, X in plan.items():
    want = ORIG["meshes"][M]["mats"]
    used = {p.material_index for p in X.polygons}
    if mat_names(X) != want or len(used) != len(want):
        bad.append([M, mat_names(X), sorted(used)])
    if X.name != M:
        bad.append([M, "name", X.name])
    if sorted(o.name for o in objs if o.type == 'MESH' and o.data == X) != sorted(o.name for o in users_of[M]):
        bad.append([M, "users differ"])
rep["final_check_bad"] = bad
rep["summary"] = {"meshes": len(plan), "renamed": rep["renamed"], "copied": len(rep["copied"]),
                  "remapped_meshes": len(rep["remapped_faces"]), "slot_keepers": sum(len(v) for v in rep["slot_keepers"].values()),
                  "added_objects": len(rep["added_objects"]), "missing_objects": len(rep["missing_objects"]),
                  "transform_diffs": len(rep["transform_diffs"]), "parent_diffs": len(rep["parent_diffs"]),
                  "errors": len(rep["errors"]), "bad": len(bad)}
json.dump(rep, open(REPORT, "w"), indent=1)
print("RESTORE", json.dumps(rep["summary"]))
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
