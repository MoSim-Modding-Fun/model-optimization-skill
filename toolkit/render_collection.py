# Render a collection asset (or every visible mesh outside HP/LP/Cutters/REVIEW) from iso / iso_back / side / under / close views.
# Used to compare a new asset with the reference asset (e.g. SDS MK4I) and the CAD:
#   blender -b file.blend --python render_collection.py -- <out prefix> ["<collection name>"]
import bpy, sys
from mathutils import Vector
a = sys.argv[sys.argv.index("--") + 1:]
pre = a[0]; coll = a[1] if len(a) > 1 else None
sc = bpy.context.scene
if coll:
    c = bpy.data.collections[coll]
    show = set(o.name for o in c.all_objects if o.type == 'MESH')
else:
    show = set(o.name for o in sc.objects if o.type == 'MESH' and not o.hide_render and not any(cc.name.startswith(("REVIEW", "HP", "Cutters", "LP")) for cc in o.users_collection))
for o in sc.objects:
    if o.type == 'MESH': o.hide_render = o.name not in show
for cc in bpy.data.collections:
    cc.hide_render = False
for lc in bpy.context.view_layer.layer_collection.children:
    lc.exclude = False
sc.render.engine = 'BLENDER_WORKBENCH'; sc.render.resolution_x = sc.render.resolution_y = 900
sh = sc.display.shading; sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'; sh.show_cavity = False; sh.show_object_outline = False
sc.display.render_aa = '8'; sc.view_settings.view_transform = 'Standard'
for m in bpy.data.materials:
    if m.use_nodes and m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled' and not n.inputs['Base Color'].is_linked:
                m.diffuse_color = (*n.inputs['Base Color'].default_value[:3], 1)
objs = [sc.objects[n] for n in show]
bb = [o.matrix_world @ Vector(c) for o in objs for c in o.bound_box]
mn = Vector((min(v.x for v in bb), min(v.y for v in bb), min(v.z for v in bb))); mx = Vector((max(v.x for v in bb), max(v.y for v in bb), max(v.z for v in bb)))
ctr = (mn + mx) / 2; R = (mx - mn).length / 2
cam = bpy.data.cameras.new("rc"); co = bpy.data.objects.new("rc", cam); sc.collection.objects.link(co); sc.camera = co
cam.lens = 50; cam.clip_start = 0.001; cam.clip_end = 50
views = {"iso": (1, -1.25, 0.85), "iso_back": (-1.1, 1.2, 0.9), "side": (1, 0, 0.1), "under": (0.4, -0.5, -0.9), "close": None}
for name, d in views.items():
    if d is None:
        d = Vector((0.9, -1.0, 0.7)).normalized(); co.location = ctr + d * R * 1.5
    else:
        d = Vector(d).normalized(); co.location = ctr + d * R * 2.6
    co.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = f"{pre}_{name}.png"
    bpy.ops.render.render(write_still=True)
print("RC", len(show), "objects")
