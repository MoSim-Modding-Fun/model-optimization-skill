# See-through check for a remodelled asset (cameras etc.; owner: "no gaps"): render the object flat white on black from
# the 26 directions of a 3x3x3 grid, once with backface culling off and once on. Unity culls back faces, so any pixel
# that differs is a spot where you look through the outside into a back face, i.e. a hole. 0 differing pixels = no gaps.
#   blender -b file.blend --python gapcheck.py -- "<object>" <out dir> [res=400] [thr_px=0]
#   (a library asset: open its .blend, or append it into an empty file first)
# Writes g<NN>_0.png (unculled) / g<NN>_1.png (culled) per view and g<NN>_diff.png (differing pixels red) for views with
# gaps. Prints "GAP dir ... pixels N" per bad view and "GAP total ..."; exit code 1 when the total exceeds thr_px.
import bpy, sys, os, itertools
import numpy as np
from mathutils import Vector, Matrix
a = sys.argv[sys.argv.index("--") + 1:]
name, wd = a[0], a[1]
res = int(a[2]) if len(a) > 2 else 400
thr = int(a[3]) if len(a) > 3 else 0
os.makedirs(wd, exist_ok=True)
o = bpy.data.objects[name]
for lc in bpy.context.view_layer.layer_collection.children:
    lc.exclude = False; lc.hide_viewport = False
for x in bpy.data.objects:
    x.hide_render = x is not o
o.hide_render = False
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'
sh = sc.display.shading
sh.light = 'FLAT'; sh.color_type = 'SINGLE'; sh.single_color = (1, 1, 1)
sh.show_cavity = False; sh.show_object_outline = False; sh.show_specular_highlight = False; sh.show_shadows = False
sh.background_type = 'VIEWPORT'; sh.background_color = (0, 0, 0)
sc.display.render_aa = 'OFF'
sc.render.resolution_x = sc.render.resolution_y = res; sc.render.resolution_percentage = 100
sc.render.image_settings.file_format = 'PNG'; sc.view_settings.view_transform = 'Standard'
dg = bpy.context.evaluated_depsgraph_get()
me = o.evaluated_get(dg).to_mesh()
co = np.array([o.matrix_world @ v.co for v in me.vertices])
o.evaluated_get(dg).to_mesh_clear()
lo, hi = co.min(0), co.max(0)
c = Vector((lo + hi) / 2); size = float(np.linalg.norm(hi - lo))
cam = bpy.data.objects.new("__gc", bpy.data.cameras.new("__gc")); sc.collection.objects.link(cam); sc.camera = cam
cam.data.type = 'ORTHO'; cam.data.ortho_scale = size * 1.1
cam.data.clip_start = size * 0.01; cam.data.clip_end = size * 10
dirs = [Vector(d).normalized() for d in itertools.product((-1, 0, 1), repeat=3) if any(d)]
bad, report = 0, []
for i, d in enumerate(dirs):
    f = -d
    up = Vector((0, 0, 1)) if abs(f.z) < 0.99 else Vector((0, 1, 0))
    rg = f.cross(up).normalized(); u = rg.cross(f)
    cam.matrix_world = Matrix((rg, u, -f)).transposed().to_4x4()
    cam.location = c + d * size * 2
    px = []
    for cull in (False, True):
        sh.show_backface_culling = cull
        fp = os.path.join(wd, f"g{i:02d}_{int(cull)}.png"); sc.render.filepath = fp
        bpy.ops.render.render(write_still=True)
        im = bpy.data.images.load(fp)
        px.append(np.array(im.pixels[:]).reshape(res, res, 4)[:, :, 0]); bpy.data.images.remove(im)
    mask = np.abs(px[0] - px[1]) > 0.02
    n = int(mask.sum()); bad += n
    report.append({"dir": [round(x, 2) for x in d], "pixels": n})
    if n:
        print("GAP dir", [round(x, 2) for x in d], "pixels", n, flush=True)
        img = bpy.data.images.new("__diff", res, res)
        rgba = np.zeros((res, res, 4), np.float32); rgba[:, :, 3] = 1
        rgba[:, :, 0] = np.maximum(px[1] * 0.35, mask); rgba[:, :, 1] = px[1] * 0.35; rgba[:, :, 2] = px[1] * 0.35
        img.pixels = rgba.ravel(); img.filepath_raw = os.path.join(wd, f"g{i:02d}_diff.png"); img.file_format = 'PNG'; img.save()
        bpy.data.images.remove(img)
print("GAP total differing pixels over", len(dirs), "views:", bad, flush=True)
import json
json.dump({"object": name, "views": report, "total": bad}, open(os.path.join(wd, "gapcheck.json"), "w"), indent=1)
sys.exit(1 if bad > thr else 0)
