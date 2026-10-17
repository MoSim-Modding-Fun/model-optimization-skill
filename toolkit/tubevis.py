# Highlight one part (lime) in the assembled robot from 8 directions. Args: <object name> <out prefix>
import bpy, sys, math
from mathutils import Vector
name = sys.argv[sys.argv.index("--")+1]; out = sys.argv[-1]
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'; sc.render.resolution_x = 700; sc.render.resolution_y = 560
sh = sc.display.shading; sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'; sh.show_cavity = False
sc.display.render_aa = '8'
hl = bpy.data.materials.new("HL"); hl.diffuse_color = (0.25, 1, 0.1, 1)  # lime: robots are often red
t = sc.objects[name]
for i in range(len(t.material_slots)): t.material_slots[i].link = 'OBJECT'; t.material_slots[i].material = hl
if not t.material_slots: t.data.materials.append(hl)
for c in bpy.data.collections:
    if c.name.startswith("REVIEW"): c.hide_render = True
bb = [t.matrix_world @ Vector(c) for c in t.bound_box]; ctr = sum(bb, Vector())/8
R = max((b - ctr).length for b in bb); dist = max(0.3, 3.5 * R)  # frame the part, not the robot
cam = bpy.data.cameras.new("c"); co = bpy.data.objects.new("c", cam); sc.collection.objects.link(co); sc.camera = co; cam.lens = 50
views = [(1,0,0.3),(-1,0,0.3),(0,1,0.3),(0,-1,0.3),(0.7,0.7,0.8),(-0.7,-0.7,0.8),(0,0,1),(0.7,-0.7,0.15)]
for i, d in enumerate(views):
    d = Vector(d).normalized(); co.location = ctr + d*dist
    co.rotation_euler = (-d).to_track_quat('-Z','Y').to_euler()
    sc.render.filepath = f"{out}_{i}.png"; bpy.ops.render.render(write_still=True)
