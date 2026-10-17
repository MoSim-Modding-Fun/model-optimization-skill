# Which object is at given pixels of a compare.py view: an object-id render (flat unique colours) from the same camera,
# REVIEW collections included. Settles "what is that dark area" claims in seconds.
#   ROBOT=<config> blender -b file.blend --python pick.py -- <view name in cfg.VIEWS> x,y [x,y ...] [--res 1400x1100]
# Pixel coordinates are in the compare.py render (origin top-left). The id render is written next to this run's file
# as pick_<view>.png.
import bpy, sys, os
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import robots
from mathutils import Vector
cfg = robots.load()
a = sys.argv[sys.argv.index("--") + 1:]
res = (1400, 1100)
if "--res" in a:
    res = tuple(int(v) for v in a[a.index("--res") + 1].split("x"))
    del a[a.index("--res"):a.index("--res") + 2]
view, pix = a[0], [tuple(map(int, p.split(","))) for p in a[1:]]
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'
sc.render.resolution_x, sc.render.resolution_y = res
sh = sc.display.shading
sh.light = 'FLAT'
sh.color_type = 'OBJECT'
sh.show_cavity = False
sc.display.render_aa = 'OFF'
sc.view_settings.view_transform = 'Raw'
ms = [o for o in sc.objects if o.type == 'MESH']
assert len(ms) < 65535, "16-bit ids"
for i, o in enumerate(ms):
    k = i + 1
    o.color = ((k & 255) / 255, ((k >> 8) & 255) / 255, 0.5, 1)
for c in bpy.data.collections:
    c.hide_render = False
cam = bpy.data.cameras.new("pick")
co = bpy.data.objects.new("pick", cam)
sc.collection.objects.link(co)
sc.camera = co
cam.lens, cam.clip_start, cam.clip_end = 60, 0.01, 100
tgt, dirv, dist = cfg.VIEWS[view]
t, d = Vector(tgt), Vector(dirv).normalized()
co.location = t + d * dist
co.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
sc.render.filepath = os.path.join(os.path.dirname(bpy.data.filepath) or ".", f"pick_{view}.png")
bpy.ops.render.render(write_still=True)
img = bpy.data.images.load(sc.render.filepath)
img.colorspace_settings.name = 'Non-Color'
W, H = img.size
px = img.pixels[:]
for (x, y) in pix:
    j = ((H - 1 - y) * W + x) * 4
    k = round(px[j] * 255) + (round(px[j + 1] * 255) << 8)
    o = ms[k - 1] if 0 < k <= len(ms) else None
    print("PICK", (x, y), o.name if o else None, [c.name for c in o.users_collection] if o else None)
