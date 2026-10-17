# A library asset next to its CAD (the file's HP collection), same cameras, Workbench, cavity off,
# backface culling on (Unity culls). CAD custom normals are dropped (Onshape glTF normals are often broken).
#   blender -b asset.blend --python kit_cmp.py -- <out prefix> ['[[x,y,z], ...]' view directions] ['{"target": [x,y,z], "r": m}']
#   (the optional third argument frames a close-up: target point in the asset frame, framing radius in metres)
import bpy, sys, math, json
import numpy as np
from mathutils import Vector
a = sys.argv[sys.argv.index("--") + 1:]
OUT = a[0]
DIRS = json.loads(a[1]) if len(a) > 1 else [[1, -1, 0.8], [-1, 1, 0.6], [0.05, 0.05, 1], [1, 0.3, -0.7]]
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'; sc.render.resolution_x = sc.render.resolution_y = 560
sh = sc.display.shading; sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'; sh.show_cavity = False
sh.show_backface_culling = True; sh.show_object_outline = False
sc.display.render_aa = '8'; sc.view_settings.view_transform = 'Standard'
for m in list(bpy.data.materials):  # linked library finishes are read-only: make them local to set viewport colours
    if m.library:
        m.make_local()
for m in bpy.data.materials:
    if m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled' and not n.inputs['Base Color'].is_linked:
                m.diffuse_color = (*n.inputs['Base Color'].default_value[:3], 1); m.metallic = 0; m.roughness = 0.5
for c in bpy.context.view_layer.layer_collection.children:
    c.exclude = False
for c in bpy.data.collections:
    c.hide_render = False
hp = [o for o in bpy.data.collections["HP"].all_objects if o.type == 'MESH']
asset = [o for o in bpy.data.collections["Asset"].all_objects if o.type == 'MESH']
for o in hp:
    me = o.data
    if "custom_normal" in me.attributes:
        me.attributes.remove(me.attributes["custom_normal"])
    me.shade_smooth(); me.set_sharp_from_angle(angle=math.radians(30))
cam_d = bpy.data.cameras.new("c"); cam_d.lens = 50; cam_d.clip_start = 0.001
cam = bpy.data.objects.new("c", cam_d); sc.collection.objects.link(cam); sc.camera = cam
pts = np.array([list(o.matrix_world @ Vector(c)) for o in hp for c in o.bound_box])
c0 = (pts.min(0) + pts.max(0)) / 2; r = np.linalg.norm(pts.max(0) - pts.min(0)) / 2
if len(a) > 2:
    zoom = json.loads(a[2]); c0 = np.array(zoom["target"]); r = zoom["r"]
for tag, show in (("cad", hp), ("asset", asset)):
    for o in hp + asset:
        o.hide_render = o not in show
    for i, d in enumerate(DIRS):
        d = Vector(d).normalized(); cam.location = Vector(c0) + d * r * 3.3
        cam.rotation_euler = (-d).to_track_quat('-Z', 'Y' if abs(d.z) < 0.95 else 'X').to_euler()
        sc.render.filepath = f"{OUT}_{i}_{tag}.png"; bpy.ops.render.render(write_still=True)
