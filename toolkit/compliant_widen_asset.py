# A 2 in wide version of the library's 2.25 in compliant wheel (16-sided): straight compliant wheels have the same face
# pattern at every width, so each colour variant is the 1 in asset stretched along its axis (z, centred). HP = this
# robot's CAD wheel (2 in wide, hex hub insert) in the same frame; LP / Cutters scaled alike; previews re-rendered.
#   bl.sh "<library 1in 16-sided .blend>" --python widen_wheel.py -- <out library .blend> <cad wheel .blend> <scale>
import bpy, os, sys
from mathutils import Matrix, Vector
a = sys.argv[sys.argv.index("--") + 1:]
OUT, CADW, S = a[0], a[1], float(a[2])
if os.path.exists(OUT):
    raise SystemExit(f"refusing to overwrite {OUT}")
Z = Matrix.Diagonal((1, 1, S, 1))
for me in {o.data for o in bpy.data.objects if o.type == 'MESH' and o.data.library is None}:
    me.transform(Z)       # each mesh once (shared meshes would be scaled twice)
for o in bpy.data.objects:
    if o.asset_data:
        new = o.name.replace("1in WD", "2in WD")
        o.name = o.data.name = new
        o.asset_data.description = (f"{new}: the library 2.25in 1in-wide 16-sided compliant wheel stretched to 2in along its "
                                    f"axis (same face pattern and tris); built for a robot with 2in wide 2.25in compliant wheels")
# HP: the robot's CAD wheel instead of the vendor's 1in wheel
hp = bpy.data.collections.get("HP")
for o in list(hp.objects):
    bpy.data.objects.remove(o)
with bpy.data.libraries.load(CADW, link=False) as (src, dst):
    dst.objects = list(src.objects)
for o in dst.objects:
    hp.objects.link(o)
# previews
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'; sc.render.resolution_x = sc.render.resolution_y = 256; sc.render.film_transparent = True
sh = sc.display.shading; sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'; sh.show_cavity = False
cam = bpy.data.objects.new("__pv", bpy.data.cameras.new("__pv")); sc.collection.objects.link(cam); sc.camera = cam
cam.data.type = 'ORTHO'; cam.data.ortho_scale = 0.095; cam.data.clip_end = 2
d = Vector((1, -1.1, 0.9)).normalized(); cam.location = d * 0.5; cam.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
assets = [o for o in bpy.data.objects if o.asset_data]
for lc in bpy.context.view_layer.layer_collection.children:
    if lc.name == "HP":
        lc.exclude = True
for o in assets:
    for x in sc.objects:
        x.hide_render = x is not o and x is not cam
    pv = os.path.join(os.path.dirname(os.path.abspath(CADW)), o.name.split(" ")[0] + "_preview.png")
    sc.render.filepath = pv
    bpy.ops.render.render(write_still=True)
    with bpy.context.temp_override(id=o):
        bpy.ops.ed.lib_id_load_custom_preview(filepath=pv)
bpy.data.objects.remove(cam)
for x in sc.objects:
    x.hide_render = False
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True, relative_remap=True)
for o in assets:
    zs = [v.co.z for v in o.data.vertices]
    print("WHEEL", o.name, "tris", sum(len(p.vertices) - 2 for p in o.data.polygons), "width mm", round((max(zs) - min(zs)) * 1000, 2), flush=True)
print("WHEEL saved", OUT, [l.filepath for l in bpy.data.libraries], flush=True)
