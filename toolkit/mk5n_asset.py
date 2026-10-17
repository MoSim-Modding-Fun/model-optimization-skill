# SDS MK5n swerve modules as library collection assets "MK5n A" (a robot's "SDS MK5n <1>"/<2>) and "MK5n B" (<3>/<4>: the
# mirrored motor layout with its own housing), laid out like the SDS MK4I / MK4n assets: <name> (empty) -> "<name> Parts" /
# "<name> Wheel" (empties, Wheel = the steerable turret + wheel) -> one low-poly object per part (origin at its centre,
# identical parts share a mesh). No motors (owner rule for every module asset). Frame as the MK4I asset: L-plate outer
# corner at the origin, z = 0 at the housing's base-plate bottom (frame-rail top), module in -x / +y (mk5n_extract.py).
# The vendor export fuses the housing and the turret into one solid each: they are Collapse-decimated (COLLAPSE: 0.075,
# owner's choice; the render-error knee was 0.2) and keep their CAD colours mapped to the closest RobotMaterials
# finish; the other parts take one finish by part type (MK4I palette).
# Input: per type the mk5n_extract work file + its pipeline run (ROBOT=mk5n MK5N_ROOT="MK5N A", robots/rmk5n.py) + vis.json.
#   blender -b --factory-startup --python mk5n_asset.py -- <work dir with A/ and B/> <out.blend>
# MK5N_TYPES (JSON list of [work-dir tag, source note, collection name, contents]) builds other module sets from the same
# layout; default = the A / B pair. Lifted turret (its own library file):
#   MK5N_TYPES='[["A", "<source> ...", "MK5n A Lifted", "..."], ["B", ...]]'
import bpy, sys, os, json, re, hashlib, collections, math
import numpy as np
from mathutils import Vector, Matrix
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import shading
WD, OUT = sys.argv[sys.argv.index("--") + 1:][:2]
import paths  # noqa: E402
RM = paths.robot_materials()
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"  # same catalog as the MK4I / MK4n assets
SCREW_MIN_PX = 400   # screw heads seen by fewer pixels (max over the vis views) are not noticeable
PART_MIN_PX = 300
# owner decision (2026-10-08): Collapse 0.075 (9.6k per module); 0.2 was the render-error knee (19.7k) but
# looks the same on the robot. MK5N_COLLAPSE (JSON) builds variants.
COLLAPSE = json.loads(os.environ.get("MK5N_COLLAPSE", "{\"Housing\": 0.075, \"Turret\": 0.075}"))
_CONTENTS = "housing, L plate, turret, molded wheel, module bolts and spacer"
TYPES = [t + [f"MK5n {t[0]}", _CONTENTS][len(t) - 2:] for t in json.loads(os.environ.get("MK5N_TYPES", json.dumps(
    [["A", "'SDS MK5n <1>' / <2>"], ["B", "'SDS MK5n <3>' / <4> (mirrored motor layout)"]])))]

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
bpy.context.preferences.filepaths.save_version = 0

FINISH = {"Plate": "Aluminum - Machined", "Spacer": "Aluminum - Machined", "Screw": "Steel - Gear"}
CAD_COLOURED = {"Housing", "Turret", "Wheel", "Screw"}  # fused / multi-colour parts: CAD colours -> closest finish;
# screws too (review: black Steel - Gear heads read as empty holes from below; the CAD heads are mid grey)
CANDS = ["Aluminum - Machined", "Aluminum - Circular Brushed", "Plastic - Light Grey", "Plastic - Grey", "Plastic - Black",
         "Plastic - White", "Steel - Plain", "Steel - Gear", "Rubber - Black", "Rubber - Colson Wheel",
         "Anodized Aluminum - Black", "Bronze - Bushing", "Anodized Aluminum - Blue", "Plastic - Red"]
with bpy.data.libraries.load(RM, link=False) as (src, dst):
    dst.materials = sorted(set(FINISH.values()) | set(CANDS) | {"Powder Coat Aluminum - White", "Rubber - Black", "Rubber - Colson Wheel", "Plastic - White"})
LIBM = {m.name: m for m in dst.materials if m}


def base_colour(m):
    if m and m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled':
                return np.array(n.inputs['Base Color'].default_value[:3])
    return np.array(m.diffuse_color[:3]) if m else np.array([0.8, 0.8, 0.8])


# the molded wheel is plastic/rubber: non-metallic finishes only (a metallic aluminium finish renders the light CAD wheel
# dark in Blender's viewport, though Unity only reads the base colour)
WHEEL_CANDS = ["Powder Coat Aluminum - White", "Plastic - White", "Plastic - Light Grey", "Plastic - Grey", "Plastic - Black",
               "Rubber - Black", "Rubber - Colson Wheel"]


def closest_finish(m, cands=CANDS):
    c = np.sqrt(np.clip(base_colour(m), 0, 1))  # roughly perceptual (cam_asset.lib_for)
    return LIBM[min(cands, key=lambda n: np.linalg.norm(np.sqrt(np.clip(base_colour(LIBM[n]), 0, 1)) - c))]


def classify(name):
    """(category, kind) from the CAD part name; category 'out' = not in the asset."""
    if "Kraken" in name:
        return "motor", None
    if name.startswith("SDS MK5n Top Assembly"):
        return ("part", "Housing") if name.endswith("~000") else ("out", None)   # the other islands sit inside
    if name.startswith(("SDS MK5 Turret Assembly", "MK5 Lifted Turret Assembly")):   # the lifted revision
        return ("part", "Turret") if name.endswith("~000") else ("out", None)    # ~001: label disc, hidden
    if name.startswith("SDS MK5 Molded Wheel"):
        return "part", "Wheel"
    if name.startswith("SDS MK5 L Plate"):
        return "part", "Plate"
    if re.search(r"SHCS|BHCS", name):
        return "screw", "Screw"
    if "Spacer" in name:
        return "part", "Spacer"
    return "part", "Part"


def baked_mesh(o, kind):
    """Evaluated mesh (Collapse at the asset ratio, Planar + Triangulate applied) without the Smooth by Angle /
    WeightedNormal modifiers, which are re-added live on the asset object."""
    keep = []
    for m in o.modifiers:
        if m.type == 'WEIGHTED_NORMAL' or (m.type == 'NODES' and m.node_group and m.node_group.name.startswith(shading.SBA)):
            keep.append(m); m.show_viewport = m.show_render = False
        elif m.type == 'DECIMATE' and m.decimate_type == 'COLLAPSE' and kind in COLLAPSE:
            m.ratio = COLLAPSE[kind]
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(o.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
    me.transform(o.matrix_world)
    for m in keep:
        m.show_viewport = m.show_render = True
    import build
    nflip = build.fix_inverted_shells(me)  # review 2: small closed islands of the decimated housing came out inside-out
    print("BAKE", o.name, kind, "inside-out shells flipped:", nflip, flush=True)
    return me, bool(keep)


# owner (2026-10-08): "you dont need decals on the swerve covers. you can just have it be a circle (for the sds mk5n)":
# the round top cover with the embossed SDS logo (CAD colour 0.6 grey, ~54 mm) becomes a plain disc
COVER_MAT = "0.600000_0.600000_0.600000"


def plain_cover(me, sides=32):
    """Delete the housing's cover faces (CAD material COVER_MAT) and put a plain closed disc of the same footprint (max
    radius of those faces about their centre, their z range) in that material. Returns (faces removed, radius mm)."""
    import bmesh
    slot = next((i for i, m in enumerate(me.materials) if m and m.name.startswith(COVER_MAT)), None)
    if slot is None:
        return 0, 0.0
    bm = bmesh.new(); bm.from_mesh(me)
    cov = [f for f in bm.faces if f.material_index == slot]
    if not cov:
        bm.free(); return 0, 0.0
    P = np.array([v.co[:] for f in cov for v in f.verts])
    cx, cy = (P[:, 0].min() + P[:, 0].max()) / 2, (P[:, 1].min() + P[:, 1].max()) / 2
    r = float(np.hypot(P[:, 0] - cx, P[:, 1] - cy).max())
    z0, z1 = float(P[:, 2].min()), float(P[:, 2].max())
    n = len(cov)
    bmesh.ops.delete(bm, geom=cov, context='FACES')
    ang = np.arange(sides) * 2 * math.pi / sides
    bot = [bm.verts.new((cx + r * math.cos(t), cy + r * math.sin(t), z0)) for t in ang]
    top = [bm.verts.new((cx + r * math.cos(t), cy + r * math.sin(t), z1)) for t in ang]
    new = [bm.faces.new(top), bm.faces.new(bot[::-1])]
    for i in range(sides):
        j = (i + 1) % sides
        new.append(bm.faces.new((bot[i], bot[j], top[j], top[i])))
    for f in new:
        f.material_index = slot
        f.smooth = True
    bmesh.ops.recalc_face_normals(bm, faces=new)
    bm.to_mesh(me); bm.free(); me.update()
    return n, r * 1000


def set_finishes(me, kind):
    if kind in CAD_COLOURED:
        slots, remap = [], []
        for m in me.materials:
            L = closest_finish(m, WHEEL_CANDS if kind == "Wheel" else CANDS)
            if kind == "Housing" and m and m.name.startswith(COVER_MAT):
                L = LIBM["Aluminum - Circular Brushed"]  # neutral grey like the CAD cover (closest was tan Plastic - White)
            if L not in slots:
                slots.append(L)
            remap.append(slots.index(L))
        mi = np.empty(len(me.polygons), np.int32); me.polygons.foreach_get("material_index", mi)
        me.materials.clear()
        for L in slots:
            me.materials.append(L)
        me.polygons.foreach_set("material_index", np.array(remap, np.int32)[np.clip(mi, 0, len(remap) - 1)])
    else:
        me.materials.clear()
        me.materials.append(LIBM[FINISH.get(kind, "Aluminum - Machined")])
        me.polygons.foreach_set("material_index", np.zeros(len(me.polygons), np.int32))


summary = {}
for tag, vendor, NAME, contents in TYPES:
    D = os.path.join(WD, tag)
    rep = {r["object"]: r for r in json.load(open(os.path.join(D, "report.json")))["rows"]}
    px = json.load(open(os.path.join(D, "vis.json")))["passes"]["full"]["px"]
    with bpy.data.libraries.load(os.path.join(D, f"mk5n_{tag}_simplified.blend"), link=False) as (src, dst):
        dst.objects = [n for n in src.objects]
    pieces = [o for o in dst.objects if o and o.type == 'MESH']
    for o in pieces:
        sc.collection.objects.link(o)
    root = bpy.data.objects.new(NAME, None)
    grp = {g: bpy.data.objects.new(f"{NAME} {g}", None) for g in ("Parts", "Wheel")}
    for g in grp.values():
        g.parent = root
    asset_objs = [root] + list(grp.values())
    shared, counts, log = {}, collections.Counter(), []
    for o in sorted(pieces, key=lambda x: x.name):
        cat, kind = classify(o.name)
        r = rep.get(o.name, {})
        v = px.get(o.name, [0])[0]
        why = cat
        if cat in ("part", "screw") and r.get("review"):
            cat, why = "out", r["review"]
        elif cat == "screw" and v < SCREW_MIN_PX:
            cat, why = "out", f"screw, {v} px (not noticeable)"
        elif cat == "part" and v < PART_MIN_PX:
            cat, why = "out", f"{v} px (not noticeable)"
        log.append({"piece": o.name, "cat": cat, "kind": kind, "why": why, "px": v, "method": r.get("method")})
        if cat in ("out", "motor"):
            continue
        me, shade = baked_mesh(o, kind)
        if kind == "Housing":
            nrem, rmm = plain_cover(me)
            print("COVER", NAME, "logo cover faces replaced by a plain disc:", nrem, "faces, r", round(rmm, 2), "mm", flush=True)
        set_finishes(me, kind)
        bb = np.array([list(x.co) for x in me.vertices]); centre = Vector(((bb.min(0) + bb.max(0)) / 2).tolist())
        me.transform(Matrix.Translation(-centre))
        co = np.empty(len(me.vertices) * 3); me.vertices.foreach_get("co", co)
        lv = np.empty(len(me.loops), np.int32); me.loops.foreach_get("vertex_index", lv)
        h = hashlib.sha1(np.round(co, 5).tobytes() + lv.tobytes() + kind.encode()).hexdigest()
        if h in shared:
            bpy.data.meshes.remove(me); me = shared[h]
        else:
            counts[kind] += 1; me.name = f"{NAME} - {kind}.{counts[kind]:03d}"; shared[h] = me
        counts[kind + " obj"] += 1
        ob = bpy.data.objects.new(f"{NAME} - {kind}.{counts[kind + ' obj']:03d}", me)
        ob.location = centre
        ob.parent = grp["Wheel" if kind in ("Turret", "Wheel") else "Parts"]
        ob["cad_part"] = o.name
        if shade:
            shading.add_shading(ob)
        asset_objs.append(ob)
    for o in pieces:
        bpy.data.objects.remove(o)
    coll = bpy.data.collections.new(NAME)
    sc.collection.children.link(coll)
    for o in asset_objs:
        coll.objects.link(o)
    coll.asset_mark()
    coll.asset_data.catalog_id = CATALOG
    coll.asset_data.description = (f"SDS MK5n swerve module, type {tag} (from {vendor}; L plate below the frame rail, housing "
                                   f"on top), low-poly in the style of the SDS MK4I / MK4n assets: {contents}. The vendor "
                                   "export fuses the housing and the turret, so those are Collapse-decimated "
                                   f"({' / '.join(sorted({str(v) for v in COLLAPSE.values()}))}). No motors: place the "
                                   "library Kraken X60 / X44 as their own objects.")
    for t in ("swerve", "SDS", "MK5n", "drivetrain"):
        coll.asset_data.tags.new(t)
    dg = bpy.context.evaluated_depsgraph_get(); tris = 0
    for o in coll.all_objects:
        if o.type == 'MESH':
            ev = o.evaluated_get(dg); m = ev.to_mesh(); tris += sum(len(p.vertices) - 2 for p in m.polygons); ev.to_mesh_clear()
    summary[NAME] = {"objects": sum(1 for o in coll.all_objects if o.type == 'MESH'), "meshes": len(shared), "tris": tris,
                     "kinds": {k: c for k, c in counts.items() if not k.endswith(" obj")}, "pieces": log}
    print("ASSET", NAME, json.dumps({k: summary[NAME][k] for k in ("objects", "meshes", "tris", "kinds")}), flush=True)

# HP: the CAD modules (work files, same frame, motors included), one joined mesh each, excluded from the view layer
hp = bpy.data.collections.new("HP")
sc.collection.children.link(hp)
for tag, vendor, NAME, _ in TYPES:
    with bpy.data.libraries.load(os.path.join(WD, tag, f"mk5n_{tag}_work.blend"), link=False) as (src, dst):
        dst.objects = [n for n in src.objects]
    # appended objects report an identity matrix_world until they are in the scene and evaluated (the first HPs
    # were every piece at its parent-local position, scattered around the asset)
    for o in dst.objects:
        if o:
            sc.collection.objects.link(o)
    bpy.context.view_layer.update()
    parts = []
    for o in dst.objects:
        if o and o.type == 'MESH':
            me = o.data.copy(); me.transform(o.matrix_world); t = bpy.data.objects.new(o.name, me)
            sc.collection.objects.link(t); parts.append(t)
    for o in dst.objects:
        if o:
            bpy.data.objects.remove(o)
    for t in parts:
        t.select_set(True)
    bpy.context.view_layer.objects.active = parts[0]
    with bpy.context.temp_override(active_object=parts[0], selected_editable_objects=parts, selected_objects=parts, object=parts[0]):
        bpy.ops.object.join()
    cad = parts[0]; cad.name = cad.data.name = f"{NAME} (vendor CAD, {vendor.split()[0]})"
    sc.collection.objects.unlink(cad); hp.objects.link(cad)
hp.hide_render = True
bpy.context.view_layer.layer_collection.children["HP"].exclude = True
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
# last: inside-out closed shells, after mesh validation (more islands close than at bake time: review 2 found 7 per asset)
import build
for c_ in [bpy.data.collections[t[2]] for t in TYPES]:
    for me_ in {o.data for o in c_.all_objects if o.type == 'MESH'}:
        me_.validate(clean_customdata=False)
        # review 3: small closed islands with MIXED winding (a screw-head cap inverted): recalc each closed island
        import bmesh as bmesh_
        bm_ = bmesh_.new(); bm_.from_mesh(me_); seen_ = set(); nre = 0
        for f0 in bm_.faces:
            if f0 in seen_:
                continue
            isl, st = [], [f0]; seen_.add(f0)
            while st:
                f = st.pop(); isl.append(f)
                for e in f.edges:
                    for g in e.link_faces:
                        if g not in seen_:
                            seen_.add(g); st.append(g)
            if len(isl) < 2000 and all(len(e.link_faces) == 2 for f in isl for e in f.edges):
                bmesh_.ops.recalc_face_normals(bm_, faces=isl); nre += 1
        bm_.to_mesh(me_); bm_.free(); me_.update()
        n_ = build.fix_inverted_shells(me_)
        if n_:
            print("FINAL inside-out shells flipped", me_.name, n_, flush=True)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
json.dump(summary, open(os.path.splitext(OUT)[0] + "_info.json", "w"), indent=1)
print("SAVED", OUT)
