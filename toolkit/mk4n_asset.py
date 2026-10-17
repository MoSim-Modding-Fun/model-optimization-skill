# SDS MK4n swerve modules as library collection assets "MK4n A" (vendor "SDS MK4n <1>") and "MK4n B" ("<2>"), laid out
# like the SDS MK4I asset: <name> (empty) -> "<name> Parts" / "<name> Wheel" (empties, Wheel = the steerable turret) ->
# one low-poly object per part (origin at its centre, identical parts share a mesh), one RobotMaterials finish per part
# by part type, HP collection with the vendor CAD module (excluded). No motors (owner): the Kraken X60s stay out.
# Frame as the MK4I asset: L-plate outer corner at the origin, base plate on z = 0, module in -x / +y.
# Input: per type, the mk4n_extract work file + its pipeline run (ROBOT=mk4n, robots/rmk4n.py) and vis.json.
#   blender -b --factory-startup --python mk4n_asset.py -- <work dir with A/ and B/> <vendor_groups.json> <out.blend>
import bpy, sys, os, json, re, hashlib, collections
import numpy as np
from mathutils import Vector, Matrix
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import shading
WD, GROUPS, OUT = sys.argv[sys.argv.index("--") + 1:][:3]
import paths  # noqa: E402
RM = paths.robot_materials()
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"  # same catalog as the MK4i asset
SCREW_MIN_PX = 400   # screw heads seen by fewer pixels (max over the vis views) are not noticeable
PART_MIN_PX = 300

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
bpy.context.preferences.filepaths.save_version = 0

# part type -> finish (MK4I asset palette)
FINISH = {"Plate": "Aluminum - Machined", "Spacer": "Aluminum - Machined", "Wheel Mount": "Aluminum - Machined",
          "Wheel Hub": "Aluminum - Machined", "Shaft": "Steel - Plain", "Gear": "Steel - Gear", "Bevel Gear": "Steel - Gear",
          "Screw": "Steel - Gear", "Steering Pulley": "Aluminum - Polished", "Belt": "Timing - HTD Belt",
          "Bearing Ring": "Steel - Machined", "Wheel Tread": "Rubber - Black", "Part": "Aluminum - Machined"}
with bpy.data.libraries.load(RM, link=False) as (src, dst):
    dst.materials = sorted(set(FINISH.values()))
LIBM = {m.name: m for m in dst.materials if m}


def classify(name):
    """(category, kind) from the vendor part name; category 'out' = not in the asset."""
    n = name
    if "Kraken" in n:
        return "motor", None
    if re.search(r"cancoder|^mesh(888|930)_mesh|Magnet|E Clip|Shim", n):
        return "out", None            # encoder + its housing, magnet, clips and shims (the MK4I leaves these out)
    if n.startswith("Wheel, SDS"):
        return "part", "Wheel Hub"    # (its name says "Bearing Bore": check it before the bearings)
    if "X-Contact Bearing" in n:
        return "part", "Bearing Ring"
    if "Bearing" in n:
        return "out", None            # small bearings
    if re.search(r"SHCS|BHCS|Socket Head Screw", n):
        return "screw", "Screw"
    if "Screw, Shoulder" in n:
        return "part", "Screw"        # wheel axle bolt
    if "Tread" in n and not n.startswith("Wheel"):
        return "part", "Wheel Tread"
    if "Wheel Mount" in n:
        return "part", "Wheel Mount"
    if "Bevel Gear" in n:
        return "part", "Bevel Gear"
    if "Pulley" in n:
        return "part", "Steering Pulley"
    if "Belt" in n:
        return "part", "Belt"
    if "Gear" in n:
        return "part", "Gear"
    if re.search(r"Shaft|Center Column", n):
        return "part", "Shaft"
    if re.search(r"Plate|Master Part|Semicircle Spacer|Support Spacer", n):
        return "part", "Plate"
    if "Spacer" in n:
        return "part", "Spacer"
    return "part", "Part"


def baked_mesh(o):
    """Evaluated mesh (Decimate applied) with the Smooth by Angle / WeightedNormal modifiers left out (re-added live)."""
    keep = []
    for m in o.modifiers:
        if m.type == 'WEIGHTED_NORMAL' or (m.type == 'NODES' and m.node_group and m.node_group.name.startswith(shading.SBA)):
            keep.append((m, m.show_viewport, m.show_render))
            m.show_viewport = m.show_render = False
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(o.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
    me.transform(o.matrix_world)
    for m, v, r in keep:
        m.show_viewport, m.show_render = v, r
    return me, bool(keep)


groups = json.load(open(GROUPS))
summary = {}
for tag, vendor_root in (("A", "SDS MK4n <1>"), ("B", "SDS MK4n <2>")):
    NAME = f"MK4n {tag}"
    D = os.path.join(WD, tag)
    turret = set(groups[vendor_root]["turret"])
    rep = {r["object"]: r for r in json.load(open(os.path.join(D, "report.json")))["rows"]}
    px = json.load(open(os.path.join(D, "vis.json")))["passes"]["full"]["px"]
    with bpy.data.libraries.load(os.path.join(D, f"mk4n_{tag}_simplified.blend"), link=False) as (src, dst):
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
        if r.get("review"):
            cat, why = "out", r["review"]
        elif cat == "screw" and v < SCREW_MIN_PX:
            cat, why = "out", f"screw, {v} px (not noticeable)"
        elif cat == "part" and v < PART_MIN_PX:
            cat, why = "out", f"{v} px (not noticeable)"
        log.append({"piece": o.name, "cat": cat, "kind": kind, "why": why, "px": v, "tris": r.get("after")})
        if cat in ("out", "motor"):
            continue
        me, shade = baked_mesh(o)
        me.materials.clear()
        me.materials.append(LIBM[FINISH[kind]])
        me.polygons.foreach_set("material_index", np.zeros(len(me.polygons), np.int32))
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
        ob.parent = grp["Wheel" if o.name in turret else "Parts"]
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
    coll.asset_data.description = (f"SDS MK4n swerve module, type {tag} (vendor CAD 'SDS MK4n <{1 if tag == 'A' else 2}>', L plate "
                                   "'Below Frame'), low-poly in the style of the SDS MK4I asset: plates, standoffs, drivetrain, "
                                   "wheel (nitrile tread) and noticeable screws; encoder, small bearings, clips and shims left out. "
                                   "No motors: place the library Kraken X60s as their own objects.")
    for t in ("swerve", "SDS", "MK4n", "drivetrain"):
        coll.asset_data.tags.new(t)
    dg = bpy.context.evaluated_depsgraph_get(); tris = 0
    for o in coll.all_objects:
        if o.type == 'MESH':
            ev = o.evaluated_get(dg); m = ev.to_mesh(); tris += sum(len(p.vertices) - 2 for p in m.polygons); ev.to_mesh_clear()
    summary[NAME] = {"objects": sum(1 for o in coll.all_objects if o.type == 'MESH'), "meshes": len(shared), "tris": tris,
                     "kinds": {k: c for k, c in counts.items() if not k.endswith(" obj")},
                     "left_out": [x for x in log if x["cat"] in ("out", "motor")]}
    print("ASSET", NAME, json.dumps({k: summary[NAME][k] for k in ("objects", "meshes", "tris", "kinds")}), flush=True)

# HP: the vendor CAD modules (work files, same frame), one joined mesh each, excluded from the view layer
hp = bpy.data.collections.new("HP")
sc.collection.children.link(hp)
for tag in ("A", "B"):
    with bpy.data.libraries.load(os.path.join(WD, tag, f"mk4n_{tag}_work.blend"), link=False) as (src, dst):
        dst.objects = [n for n in src.objects]
    ms = [o for o in dst.objects if o and o.type == 'MESH']
    dg = bpy.context.evaluated_depsgraph_get()
    parts = []
    for o in ms:
        me = o.data.copy(); me.transform(o.matrix_world); t = bpy.data.objects.new(o.name, me); sc.collection.objects.link(t); parts.append(t)
    for o in dst.objects:
        if o: bpy.data.objects.remove(o)
    for t in parts: t.select_set(True)
    bpy.context.view_layer.objects.active = parts[0]
    with bpy.context.temp_override(active_object=parts[0], selected_editable_objects=parts, selected_objects=parts, object=parts[0]):
        bpy.ops.object.join()
    cad = parts[0]; cad.name = cad.data.name = f"MK4n {tag} (vendor CAD)"
    sc.collection.objects.unlink(cad); hp.objects.link(cad)
hp.hide_render = True
bpy.context.view_layer.layer_collection.children["HP"].exclude = True
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
json.dump(summary, open(os.path.splitext(OUT)[0] + "_info.json", "w"), indent=1)
print("SAVED", OUT)
