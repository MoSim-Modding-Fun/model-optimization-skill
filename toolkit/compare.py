# Render a blend from fixed viewpoints. Args: outprefix [hide_review 0|1] [clean]  (clean = drop CAD custom normals)
import bpy, sys, math, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robots
cfg = robots.load()
from mathutils import Vector
args = sys.argv[sys.argv.index("--")+1:]
prefix = args[0]; hide_review = len(args) > 1 and args[1] == "1"
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'; sc.render.resolution_x = 1400; sc.render.resolution_y = 1100
sh = sc.display.shading; sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'; sh.show_cavity = False; sh.show_object_outline = False
sh.show_specular_highlight = True
sc.display.render_aa = '8'; sc.view_settings.view_transform = 'Standard'
# viewport colours from principled (so the original file renders with true colours too)
for m in bpy.data.materials:
    if m.use_nodes and m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled' and not n.inputs['Base Color'].is_linked:
                m.diffuse_color = (*n.inputs['Base Color'].default_value[:3], 1)
    # render-only: the synced metallic 1.0 of the Aluminum / Steel finishes draws them near-black in Workbench
    # STUDIO light (Unity reads only the base colour); part_shots.py does the same
    m.metallic = 0.0; m.roughness = 0.5
if len(args) > 2 and args[2] == "clean":
    # CAD exports often carry broken custom normals (flat plates render crumpled): show the true geometry
    for me in bpy.data.meshes:
        if "custom_normal" in me.attributes:
            me.attributes.remove(me.attributes["custom_normal"])
            me.shade_smooth()
            me.set_sharp_from_angle(angle=math.radians(30))
if hide_review:
    for c in bpy.data.collections:
        if c.name.startswith("REVIEW"):
            c.hide_render = True
cam = bpy.data.cameras.new("cc"); co = bpy.data.objects.new("cc", cam); sc.collection.objects.link(co); sc.camera = co
cam.lens = 60; cam.clip_start = 0.01; cam.clip_end = 100
views = cfg.VIEWS
for name, (tgt, dirv, dist) in views.items():
    t = Vector(tgt); d = Vector(dirv).normalized()
    co.location = t + d * dist
    co.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = f"{prefix}_{name}.png"
    bpy.ops.render.render(write_still=True)
