# Finish a built electronics asset and write it into the library (refuses to overwrite an existing file).
#   bl.sh staged.blend --python finalize_asset.py -- "<asset object>" "<library .blend path>" "<description>" "tag1,tag2"
# Drops unused material slots (indices written back), marks the asset (Components catalog, tags, description), renders a
# 256 px preview, leaves only the Asset collection visible (HP excluded, LP / Cutters hidden), purges orphans, removes
# library entries other than RobotMaterials, and saves with relative paths remapped to the library folder.
import bpy, sys, os, math
from mathutils import Vector
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import elec_asset_lib as E  # noqa: E402
a = sys.argv[sys.argv.index("--") + 1:]
NAME, OUT, DESC, TAGS = a[0], a[1], a[2], [t for t in a[3].split(",") if t]
if os.path.exists(OUT):
    raise SystemExit(f"refusing to overwrite {OUT}")
ob = bpy.data.objects[NAME]
me = ob.data
me.name = NAME
used = sorted({p.material_index for p in me.polygons})
remap = {old: new for new, old in enumerate(used)}
mats = [me.materials[i] for i in used]
idx = [remap[p.material_index] for p in me.polygons]
me.materials.clear()
for m in mats:
    me.materials.append(m)
me.polygons.foreach_set("material_index", idx)
me.update()
ob.location = (0, 0, 0)
if ob.asset_data is None:
    ob.asset_mark()
ob.asset_data.catalog_id = E.CATALOG
ob.asset_data.description = DESC
for t in TAGS:
    ob.asset_data.tags.new(t, skip_if_exists=True)
# collections as in the other electronics files
for lc in bpy.context.view_layer.layer_collection.children:
    if lc.name == "HP":
        lc.exclude = True
    elif lc.name in ("LP", "Cutters"):
        lc.hide_viewport = True
        lc.collection.hide_render = True
# preview: Workbench, material colours, iso
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'; sc.render.resolution_x = sc.render.resolution_y = 256
sc.render.film_transparent = True
sh = sc.display.shading; sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'; sh.show_cavity = False
for m in me.materials:
    if m.node_tree:
        pb = next((n for n in m.node_tree.nodes if n.bl_idname == 'ShaderNodeBsdfPrincipled'), None)
cam = bpy.data.objects.new("__pv", bpy.data.cameras.new("__pv")); sc.collection.objects.link(cam); sc.camera = cam
cam.data.type = 'ORTHO'
bb = [ob.matrix_world @ Vector(c) for c in ob.bound_box]
mn = Vector([min(v[i] for v in bb) for i in range(3)]); mx = Vector([max(v[i] for v in bb) for i in range(3)])
ctr = (mn + mx) / 2; R = (mx - mn).length / 2
d = Vector((1, -1.1, 0.9)).normalized()
cam.location = ctr + d * 4 * R; cam.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
cam.data.ortho_scale = 2.1 * R; cam.data.clip_end = 10 * R + 1
for o in sc.objects:
    o.hide_render = o is not ob and o is not cam
pv = os.path.join(os.path.dirname(os.path.abspath(bpy.data.filepath)), NAME.split(" ")[0] + "_preview.png")
sc.render.filepath = pv
bpy.ops.render.render(write_still=True)
bpy.data.objects.remove(cam)
for o in sc.objects:
    o.hide_render = False
with bpy.context.temp_override(id=ob):
    bpy.ops.ed.lib_id_load_custom_preview(filepath=pv)
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
for lib in list(bpy.data.libraries):
    if not any(getattr(i, "library", None) == lib for c in (bpy.data.materials, bpy.data.objects, bpy.data.meshes) for i in c):
        bpy.data.libraries.remove(lib)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True, relative_remap=True)
print("FINAL", NAME, "tris", E.tris(ob), "open", E.open_edges(ob), "mats", [m.name for m in me.materials],
      "libs", [l.filepath for l in bpy.data.libraries], "->", OUT, flush=True)
