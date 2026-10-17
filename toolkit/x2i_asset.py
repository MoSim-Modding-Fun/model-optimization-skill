# WCP Swerve X2i module as a library collection asset "WCP Swerve X2i" (2026-10-15), laid out like the SDS MK4I asset
# (mk4n_asset.py): <name> (empty) -> "<name> Parts" / "<name> Wheel" (empties, Wheel = the steerable turret) -> one
# low-poly object per part (origin at its centre, identical parts share a mesh), one RobotMaterials finish per part by
# part type (X2i palette: black-anodised plates, black-oxide hardware and steel gears, aluminium rotation gear, nitrile-
# black tread), HP collection with the CAD module (excluded). No motors (owner): the Kraken X60 / X44 stay out.
# Frame as the MK4I asset: tube-mount bottom plate's outer corner at the origin, frame-rail top at z = 0, module in -x/+y.
# Input: the x2i_extract work file + its pipeline run (ROBOT=x2i, robots/rx2i.py) and vis.json, all in one work dir.
#   blender -b --factory-startup --python x2i_asset.py -- <work dir> <out.blend> [asset name]
import bpy, sys, os, json, re, hashlib, collections
import numpy as np
from mathutils import Vector, Matrix
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import shading
a = sys.argv[sys.argv.index("--") + 1:]
WD, OUT = a[:2]
NAME = a[2] if len(a) > 2 else "WCP Swerve X2i"
import paths  # noqa: E402
RM = paths.robot_materials()
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"  # same catalog as the MK4i asset
SCREW_MIN_PX = int(os.environ.get("X2I_SCREW_MIN_PX", 400))   # screw heads / nuts seen by fewer px (vis full pass)
PART_MIN_PX = 300
STRICT = os.environ.get("X2I_STRICT") == "1"   # MK4I-strict variant: small bearings out even when visible

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
bpy.context.preferences.filepaths.save_version = 0

BLACK, OXIDE = "Anodized Aluminum - Black", "Steel - Black Oxide"
FINISH = {"Plate": BLACK, "Spacer": BLACK, "Wheel Mount": BLACK, "Bracket": BLACK, "Standoff": "Aluminum - Machined",
          "Shaft": OXIDE, "Gear": OXIDE, "Bevel Gear": OXIDE, "Rotation Gear": "Aluminum - Machined", "Screw": OXIDE,
          "Nut": "Steel - Plain", "Bearing": "Steel - Plain", "Bearing Ring": "Steel - Plain",
          "Wheel": "Rubber - Colson Wheel", "Wheel Hub": "Plastic - Black", "Part": BLACK}
# CAD colours (sRGB) vs the finishes: plates 0.216 = Anodized Aluminum - Black 0.212; bearings 0.70 -> Steel - Plain 0.54
# (Steel - Machined is white); X2 molded wheel: light-grey tread 0.70 -> Rubber - Colson Wheel 0.65, black core 0.176 ->
# Plastic - Black 0.182
with bpy.data.libraries.load(RM, link=False) as (src, dst):
    dst.materials = sorted(set(FINISH.values()))
LIBM = {m.name: m for m in dst.materials if m}


def classify(name):
    """(category, kind) from the vendor part name; category 'out' = not in the asset (MK4I rules)."""
    n = name
    if "Kraken" in n:
        return "motor", None
    if re.search(r"CANcoder|Magnet|SnapRing|WCP-1700-006", n):
        return "out", None            # encoder, its magnet, clips (the MK4I leaves these out)
    if "X-Contact Bearing" in n:
        return "part", "Bearing Ring"
    if "Bearing" in n:
        return ("out", None) if STRICT else ("bearing", "Bearing")   # default: kept where noticeable
    if re.search(r"SHCS|BHCS|PHCS|Screw|WCP-1740", n):
        return "screw", "Screw"
    if re.search(r"locknut|Nut", n, re.I):
        return "screw", "Nut"
    if "Molded Wheel" in n:
        return "part", "Wheel"
    if "Wheel Fork" in n:
        return "part", "Wheel Mount"
    if "Rotation Gear" in n and "88t" in n:
        return "part", "Rotation Gear"
    if "Bevel Gear Shaft" in n:
        return "part", "Bevel Gear"
    if "Bevel Gear" in n:
        return "part", "Bevel Gear"
    if re.search(r"Gear", n):
        return "part", "Gear"
    if "Threaded Hex Standoff" in n:
        return "part", "Standoff"
    if re.search(r"Spacer", n):
        return "part", "Spacer"
    if re.search(r"Plate|Base Kit|Corner Brace", n):
        return "part", "Plate"
    if "Hardware Kit" in n:
        return "part", "Spacer"      # WCP-1700-00x: steering shaft, spacers, the steering-pulley shim ring
    return "part", "Part"


def wheel_finish(me):
    """The X2 molded wheel: light-grey tread (CAD 0.70) round a black core (0.176). The section lathe gives the whole
    revolve the CAD's area-majority colour (the core), so the tread is assigned by geometry: faces on the outer band
    (centre > 0.9 R from the axis, normal across the axis). Axis = smallest-variance principal axis of the vertices."""
    co = np.array([v.co for v in me.vertices])
    c0 = co.mean(0)
    w, V = np.linalg.eigh(np.cov((co - c0).T))
    ax = V[:, 0]
    R = np.linalg.norm(np.cross(co - c0, ax), axis=1).max()
    idx = []
    for p in me.polygons:
        c = np.array(p.center) - c0
        r = np.linalg.norm(np.cross(c, ax))
        idx.append(1 if r > 0.9 * R and abs(np.dot(np.array(p.normal), ax)) < 0.5 else 0)
    return [LIBM[FINISH["Wheel Hub"]], LIBM[FINISH["Wheel"]]], idx


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


frame = json.load(open(os.path.join(WD, "x2i_work_frame.json")))
turret = set(frame["turret"])
rep = {r["object"]: r for r in json.load(open(os.path.join(WD, "report.json")))["rows"]}
px = json.load(open(os.path.join(WD, "vis.json")))["passes"]["full"]["px"]
with bpy.data.libraries.load(os.path.join(WD, "x2i_simplified.blend"), link=False) as (src, dst):
    dst.objects = [n for n in src.objects]
pieces = [o for o in dst.objects if o and o.type == 'MESH']
for o in pieces:
    sc.collection.objects.link(o)
bpy.context.view_layer.update()
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
        cat, why = "out", f"{kind.lower()}, {v} px (not noticeable)"
    elif cat in ("part", "bearing") and v < PART_MIN_PX:
        cat, why = "out", f"{v} px (not noticeable)"
    log.append({"piece": o.name, "cat": cat, "kind": kind, "why": why, "px": v, "tris": r.get("after")})
    if cat in ("out", "motor"):
        continue
    me, shade = baked_mesh(o)
    wf = wheel_finish(me) if kind == "Wheel" else None
    me.materials.clear()
    if wf:
        for m in wf[0]:
            me.materials.append(m)
        me.polygons.foreach_set("material_index", np.array(wf[1], np.int32))
    else:
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
coll.asset_data.description = ("WCP Swerve X2i module (built from " + os.environ.get("ASSET_SOURCE", "a robot CAD") + "), low-poly in the style of the SDS MK4I "
                               "asset: plates, standoffs, drivetrain gears as plain discs, the steering turret with the "
                               "molded wheel, and noticeable screws / bearings; encoder, magnet and clips left out. No motors: "
                               "place the library Kraken X60 (drive) and Kraken X44 (steer) as their own objects. Frame: "
                               "tube-mount bottom plate's outer corner at the origin, frame-rail top at z = 0, module in -x / +y.")
for t in ("swerve", "WCP", "X2i", "drivetrain"):
    coll.asset_data.tags.new(t)
dg = bpy.context.evaluated_depsgraph_get(); tris = 0
for o in coll.all_objects:
    if o.type == 'MESH':
        ev = o.evaluated_get(dg); m = ev.to_mesh(); tris += sum(len(p.vertices) - 2 for p in m.polygons); ev.to_mesh_clear()
summary = {"objects": sum(1 for o in coll.all_objects if o.type == 'MESH'), "meshes": len(shared), "tris": tris,
           "kinds": {k: c for k, c in counts.items() if not k.endswith(" obj")}, "pieces": log}
print("ASSET", NAME, json.dumps({k: summary[k] for k in ("objects", "meshes", "tris", "kinds")}), flush=True)

# HP: the CAD module (work file, same frame), one joined mesh, excluded from the view layer
hp = bpy.data.collections.new("HP")
sc.collection.children.link(hp)
with bpy.data.libraries.load(os.path.join(WD, "x2i_work.blend"), link=False) as (src, dst):
    dst.objects = [n for n in src.objects]
ms = [o for o in dst.objects if o and o.type == 'MESH']
for o in ms:
    sc.collection.objects.link(o)
bpy.context.view_layer.update()
parts = []
for o in ms:
    me = o.data.copy(); me.transform(o.matrix_world); t = bpy.data.objects.new(o.name, me); sc.collection.objects.link(t); parts.append(t)
for o in dst.objects:
    if o:
        bpy.data.objects.remove(o)
for t in parts:
    t.select_set(True)
bpy.context.view_layer.objects.active = parts[0]
with bpy.context.temp_override(active_object=parts[0], selected_editable_objects=parts, selected_objects=parts, object=parts[0]):
    bpy.ops.object.join()
cad = parts[0]; cad.name = cad.data.name = f"{NAME} (CAD)"
sc.collection.objects.unlink(cad); hp.objects.link(cad)
hp.hide_render = True
bpy.context.view_layer.layer_collection.children["HP"].exclude = True
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
json.dump(summary, open(os.path.splitext(OUT)[0] + "_info.json", "w"), indent=1)
print("SAVED", OUT)
