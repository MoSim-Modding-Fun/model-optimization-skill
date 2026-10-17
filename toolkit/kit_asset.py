# Library OBJECT asset from a per-piece simplification of one assembly or part (kit_extract.py work file
# -> vis.py -> pipeline.py with KIT_ASSET=1). The visible simplified pieces are baked (live Collapse / planar
# dissolve applied, the Smooth by Angle / WeightedNormal stack left off), joined into ONE object at the frame origin,
# their colours mapped to RobotMaterials finishes (linked from the library, like the owner's own assets), and the stack
# is added back live on the joined object. HP = the joined CAD, excluded. Writes a staging file; asset_prune.py then
# deletes every face hidden inside the asset (owner: nothing on the inside) and gap-checks it into the library.
#   blender -b --factory-startup --python kit_asset.py -- <asset work dir> <staging .blend> "<asset name>" '<opts json>'
# opts: finish {regex on piece name: finish}, cands [finishes for colour matching], drop [regex], keep [regex: kept even
# when REVIEW/hidden], min_px (default 30: pieces seen by fewer pixels in the asset's own visibility pass are dropped),
# shade (default true), desc, tags
import bpy, sys, os, json, re
import numpy as np
from mathutils import Matrix
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import shading, build, libcombo

WD, STAGE, NAME = sys.argv[sys.argv.index("--") + 1:][:3]
OPT = json.loads(sys.argv[sys.argv.index("--") + 4]) if len(sys.argv) > sys.argv.index("--") + 4 else {}
import paths  # noqa: E402
RM = paths.robot_materials()
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"  # "Components", as every library component asset
# no "Steel - Plain" (metallic, 0.25): the CAD's dark-grey plastics (motor end caps, battery mount) matched it by colour
CANDS = OPT.get("cands", ["Aluminum - Machined", "Aluminum - Plain", "Plastic - Light Grey", "Plastic - Grey",
                          "Plastic - Black", "Plastic - White", "Steel - Black Oxide", "Rubber - Black",
                          "Anodized Aluminum - Black", "Plastic - Red", "Plastic - Blue",
                          "Powder Coat Aluminum - White"])

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
bpy.context.preferences.filepaths.save_version = 0

want = sorted(set(CANDS) | set(OPT.get("finish", {}).values()))
with bpy.data.libraries.load(RM, link=True, relative=True) as (src, dst):
    missing = [n for n in want if n not in src.materials]
    dst.materials = [n for n in want if n in src.materials]
if missing:
    raise SystemExit(f"finishes not in RobotMaterials: {missing}")
LIBM = {m.name: m for m in dst.materials}


def base_colour(m):
    if m and m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled':
                return np.array(n.inputs['Base Color'].default_value[:3])
    return np.array(m.diffuse_color[:3]) if m else np.array([0.8, 0.8, 0.8])


def closest_finish(m):
    c = np.sqrt(np.clip(base_colour(m), 0, 1))  # roughly perceptual (cam_asset.lib_for)
    return LIBM[min(CANDS, key=lambda n: np.linalg.norm(np.sqrt(np.clip(base_colour(LIBM[n]), 0, 1)) - c))]


def forced_finish(piece):
    for rx, fin in OPT.get("finish", {}).items():
        if re.search(rx, piece):
            return LIBM[fin]
    return None


rep = {r["object"]: r for r in json.load(open(os.path.join(WD, "report.json")))["rows"]}
px = json.load(open(os.path.join(WD, "vis.json")))["passes"]["full"]["px"]
with bpy.data.libraries.load(os.path.join(WD, "simplified.blend"), link=False) as (src, dst):
    dst.objects = list(src.objects)
for o in dst.objects:
    if o:
        sc.collection.objects.link(o)
bpy.context.view_layer.update()
MIN_PX = OPT.get("min_px", 30)
parts, log = [], []
for o in sorted([o for o in dst.objects if o and o.type == 'MESH'], key=lambda x: x.name):
    r = rep.get(o.name, {})
    v = px.get(o.name, [0])[0]
    keep_rx = any(re.search(rx, o.name) for rx in OPT.get("keep", []))
    why = None
    if any(re.search(rx, o.name) for rx in OPT.get("drop", [])):
        why = "dropped (opts)"
    elif r.get("review") and not keep_rx:
        why = r["review"]
    elif v < MIN_PX and not keep_rx:
        why = f"{v} px (not noticeable)"
    if why:
        log.append({"piece": o.name, "out": why, "px": v, "method": r.get("method")})
        continue
    keep = []
    for m in o.modifiers:
        if m.type == 'WEIGHTED_NORMAL' or (m.type == 'NODES' and m.node_group and m.node_group.name.startswith(shading.SBA)):
            keep.append(m); m.show_viewport = m.show_render = False
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(o.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
    me.transform(o.matrix_world)
    ff = forced_finish(o.name)
    slots = [ff or closest_finish(m) for m in me.materials] or [ff or LIBM[CANDS[0]]]
    me.materials.clear()
    for s in slots:
        me.materials.append(s)
    parts.append((me, Matrix.Identity(4)))
    log.append({"piece": o.name, "in": [s.name for s in slots], "px": v, "method": r.get("method"),
                "tris": sum(len(p.vertices) - 2 for p in me.polygons)})
for o in list(dst.objects):
    if o:
        bpy.data.objects.remove(o)
if not parts:
    raise SystemExit("no visible pieces")
me = libcombo.join_meshes(NAME, parts)
for p_, _ in parts:
    bpy.data.meshes.remove(p_)
# one slot per finish (join merges by datablock already); drop unused slots
me.validate(clean_customdata=False)
# hairline cracks (single see-through pixels) where Collapse-decimated faces meet: split T-junctions and weld
n_open0 = build.open_edges(me)
if n_open0:
    build.fix_t_junctions(me, fill=False)
nflip = build.fix_inverted_shells(me)
ob = bpy.data.objects.new(NAME, me)
coll = bpy.data.collections.new("Asset"); sc.collection.children.link(coll); coll.objects.link(ob)
if OPT.get("shade", True):
    for f in me.polygons:
        f.use_smooth = True
    shading.add_shading(ob)
ob.asset_mark()
ob.asset_data.catalog_id = CATALOG
ob.asset_data.description = OPT.get("desc", NAME)
for t in OPT.get("tags", ["component"]):
    ob.asset_data.tags.new(t)

# HP: the CAD (work file, same frame), one joined mesh, excluded from the view layer
with bpy.data.libraries.load(os.path.join(WD, "work.blend"), link=False) as (src, dst):
    dst.objects = list(src.objects)
for o in dst.objects:
    if o:
        sc.collection.objects.link(o)
bpy.context.view_layer.update()
cad = [(o.data, o.matrix_world.copy()) for o in dst.objects if o and o.type == 'MESH']
hpme = libcombo.join_meshes(NAME + " (CAD)", cad)
for o in list(dst.objects):
    if o:
        bpy.data.objects.remove(o)
hp = bpy.data.collections.new("HP"); sc.collection.children.link(hp)
hp.objects.link(bpy.data.objects.new(NAME + " (CAD)", hpme))
hp.hide_render = True
bpy.context.view_layer.layer_collection.children["HP"].exclude = True
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=False, do_recursive=True)
tris = sum(len(p.vertices) - 2 for p in me.polygons)
cadtris = sum(len(p.vertices) - 2 for p in hpme.polygons)
info = {"asset": NAME, "tris": tris, "cad_tris": cadtris, "finishes": [m.name for m in me.materials], "inside_out_flipped": nflip,
        "pieces": log}
json.dump(info, open(os.path.splitext(STAGE)[0] + "_info.json", "w"), indent=1)
print("ASSET", NAME, "tris", tris, "from CAD", cadtris, "pieces in", sum(1 for x in log if "in" in x), "out",
      sum(1 for x in log if "out" in x), "finishes", info["finishes"], flush=True)
os.makedirs(os.path.dirname(STAGE), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=STAGE, compress=True)
