# Visibility pass: unique flat colour per mesh object, ortho renders from a Fibonacci sphere of directions.
import bpy, numpy as np, json, sys, math, os, time
from mathutils import Vector
SCR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCR)
import robots
cfg = robots.load()
group_of = cfg.group_of
args = sys.argv[sys.argv.index("--")+1:]
OUT = args[0]; NVIEWS = int(args[1]); RES = int(args[2])
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'
sc.render.resolution_x = sc.render.resolution_y = RES
sc.render.film_transparent = True
sc.render.dither_intensity = 0
sc.display.render_aa = 'OFF'
sh = sc.display.shading
sh.light = 'FLAT'; sh.color_type = 'OBJECT'
for a in ("show_object_outline","show_cavity","show_shadows","show_specular_highlight","show_backface_culling","show_xray"):
    if hasattr(sh, a): setattr(sh, a, False)
sc.view_settings.view_transform = 'Raw'; sc.view_settings.look = 'None'; sc.view_settings.exposure = 0; sc.view_settings.gamma = 1
sc.render.image_settings.file_format = 'PNG'; sc.render.image_settings.color_mode = 'RGBA'; sc.render.image_settings.color_depth = '8'
cam = bpy.data.cameras.new("viscam"); co = bpy.data.objects.new("viscam", cam); sc.collection.objects.link(co); sc.camera = co
cam.type = 'ORTHO'
meshes = [o for o in sc.objects if o.type == 'MESH']
idx = {o.name: i+1 for i, o in enumerate(meshes)}
def enc(i): return ((i & 15)*16+8)/255, (((i>>4)&15)*16+8)/255, (((i>>8)&15)*16+8)/255
for o in meshes: o.color = (*enc(idx[o.name]), 1)
names = [None] + [o.name for o in meshes]

groups = {}
for o in meshes: groups.setdefault(group_of(o), []).append(o)

def fib(n):
    pts = []
    for k in range(n):
        z = 1 - 2*(k+0.5)/n; r = math.sqrt(1-z*z); th = k*math.pi*(3-math.sqrt(5))
        pts.append(Vector((r*math.cos(th), r*math.sin(th), z)))
    return pts
dirs = fib(NVIEWS)
tmp = OUT + ".tmp.png"
dg = bpy.context.evaluated_depsgraph_get()
def world_bounds(objs):
    mn = Vector((1e9,)*3); mx = Vector((-1e9,)*3)
    for o in objs:
        for c in o.bound_box:
            w = o.matrix_world @ Vector(c)
            mn = Vector(map(min, mn, w)); mx = Vector(map(max, mx, w))
    return mn, mx
def run_pass(show):
    sset = set(o.name for o in show)
    for o in meshes: o.hide_render = o.name not in sset
    mn, mx = world_bounds(show); ctr = (mn+mx)/2; R = (mx-mn).length/2
    cam.ortho_scale = 2*R*1.02; cam.clip_start = 0.01; cam.clip_end = 6*R+1
    maxpx = np.zeros(len(names), np.int64); sumpx = np.zeros(len(names), np.int64); nviews = np.zeros(len(names), np.int64)
    for d in dirs:
        co.location = ctr + d*(3*R)
        up = 'Y' if abs(d.z) < 0.99 else 'X'
        co.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
        sc.render.filepath = tmp
        bpy.ops.render.render(write_still=True)
        img = bpy.data.images.load(tmp); img.colorspace_settings.name = 'Non-Color'
        px = np.empty(RES*RES*4, np.float32); img.pixels.foreach_get(px); bpy.data.images.remove(img)
        px = px.reshape(-1, 4); m = px[:,3] > 0.5
        c = np.round(px[m,:3]*255).astype(np.int64)
        ids = (c[:,0]//16) | ((c[:,1]//16) << 4) | ((c[:,2]//16) << 8)
        cnt = np.bincount(ids, minlength=len(names))[:len(names)]
        maxpx = np.maximum(maxpx, cnt); sumpx += cnt; nviews += cnt > 0
    mm_per_px = 2*R*1.02/RES*1000
    return {names[i]: [int(maxpx[i]), int(sumpx[i]), int(nviews[i])] for i in range(1, len(names)) if names[i] in sset}, mm_per_px
res = {"groups": {g: [o.name for o in os_] for g, os_ in groups.items()}, "passes": {}}
# cfg.SEE_THROUGH_GROUPS: groups of clear parts (polycarbonate panels) that never hide anything. The full pass leaves
# them out; their pixel counts come from their own group pass (the pipeline's occluders are the "main" group anyway).
SEE = set(getattr(cfg, "SEE_THROUGH_GROUPS", ()))
t = time.time()
r, mpp = run_pass([o for o in meshes if group_of(o) not in SEE]); res["passes"]["full"] = {"px": r, "mm_per_px": mpp}
print("full done", time.time()-t, "mm/px", mpp, flush=True)
for g, os_ in groups.items():
    r, mpp = run_pass(os_); res["passes"][g] = {"px": r, "mm_per_px": mpp}
    if g in SEE:
        res["passes"]["full"]["px"].update(r)
    print(g, len(os_), "done", time.time()-t, "mm/px", mpp, flush=True)
json.dump(res, open(OUT, "w"))
