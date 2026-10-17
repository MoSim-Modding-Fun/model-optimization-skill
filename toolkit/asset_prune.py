# Hidden-geometry pruning for collection assets (MK5n; owner: "it shouldnt include anything on the inside because the
# interior is invisible due to the shell"). Every face of every mesh object in the collection gets a unique flat colour;
# orthographic renders from 100 Fibonacci-sphere directions plus the 26 grid directions, backface culling on; faces never
# seen are deleted, keeping a one-face margin around the seen ones (faces seen only through a gap were missed
# otherwise). Shared meshes take the union over their instances. Then a culled-vs-unculled gap check of the whole
# collection from the 26 grid directions: any differing pixel is a see-through spot.
#   blender -b assets.blend --python asset_prune.py -- out.blend [res=1000] "Collection A" ["Collection B" ...]
import bpy, bmesh, sys, os, math, itertools, collections
import numpy as np
from mathutils import Vector
a = sys.argv[sys.argv.index("--") + 1:]
OUT = a[0]
RES = int(a[1]) if a[1].isdigit() else 1000
COLLS = a[2:] if a[1].isdigit() else a[1:]
sc = bpy.context.scene
for lc in bpy.context.view_layer.layer_collection.children:
    if lc.name in COLLS:
        lc.exclude = False
sc.render.engine = 'BLENDER_WORKBENCH'
sh = sc.display.shading
sh.light = 'FLAT'; sh.show_cavity = False; sh.show_object_outline = False; sh.show_specular_highlight = False
sh.show_shadows = False; sh.show_xray = False
sc.display.render_aa = 'OFF'
sc.render.resolution_x = sc.render.resolution_y = RES; sc.render.resolution_percentage = 100
sc.render.film_transparent = True
sc.render.image_settings.file_format = 'PNG'; sc.render.image_settings.color_mode = 'RGBA'
sc.view_settings.view_transform = 'Raw'; sc.view_settings.look = 'None'
cam = bpy.data.objects.new("__ap", bpy.data.cameras.new("__ap")); sc.collection.objects.link(cam); sc.camera = cam
cam.data.type = 'ORTHO'
TMP = os.path.splitext(OUT)[0] + "_ap_tmp.png"


def fib(n):
    out = []
    for k in range(n):
        z = 1 - 2 * (k + 0.5) / n; r = math.sqrt(1 - z * z); th = k * math.pi * (3 - math.sqrt(5))
        out.append(Vector((r * math.cos(th), r * math.sin(th), z)))
    return out


GRID = [Vector(d).normalized() for d in itertools.product((-1, 0, 1), repeat=3) if any(d)]


def frame(objs):
    pts = np.array([o.matrix_world @ Vector(c) for o in objs for c in o.bound_box])
    lo, hi = pts.min(0), pts.max(0)
    return Vector(((lo + hi) / 2).tolist()), float(np.linalg.norm(hi - lo))


def render_px(d, c, size):
    cam.data.ortho_scale = size * 1.05; cam.data.clip_start = size * 0.01; cam.data.clip_end = size * 10
    cam.location = c + d * size * 2
    cam.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = TMP
    bpy.ops.render.render(write_still=True)
    img = bpy.data.images.load(TMP); img.colorspace_settings.name = 'Non-Color'
    px = np.empty(RES * RES * 4, np.float32); img.pixels.foreach_get(px); bpy.data.images.remove(img)
    return px.reshape(-1, 4)


def enc(i):  # 5 bits per channel, values at the bin centres (vis.py style, robust to rounding)
    return (((i & 31) * 8 + 4) / 255, (((i >> 5) & 31) * 8 + 4) / 255, (((i >> 10) & 31) * 8 + 4) / 255)


def prune(cname):
    coll = bpy.data.collections[cname]
    objs = [o for o in coll.all_objects if o.type == 'MESH']
    others = [o for o in sc.objects if o.type == 'MESH' and o not in objs]
    for o in others:
        o.hide_render = True
    # temp copies with a per-face colour id (FLOAT_COLOR on corners: linear, written raw)
    temps, base, owner = [], 1, []
    for o in objs:
        me = o.data.copy()
        n = len(me.polygons)
        col = me.color_attributes.new("fid", 'FLOAT_COLOR', 'CORNER')
        lt = np.empty(n, np.int32); me.polygons.foreach_get("loop_total", lt)
        ids = np.repeat(np.arange(base, base + n), lt)
        rgba = np.array([(*enc(int(i)), 1.0) for i in ids], np.float32)
        col.data.foreach_set("color", rgba.ravel())
        me.color_attributes.active_color = col
        t = bpy.data.objects.new("__ap_" + o.name, me); sc.collection.objects.link(t)
        t.matrix_world = o.matrix_world.copy()
        temps.append(t); owner.append((o, base, n)); base += n
        o.hide_render = True
    assert base < 32768, "too many faces for 15-bit ids"
    sh.color_type = 'VERTEX'; sh.show_backface_culling = True
    c, size = frame(temps)
    seen = np.zeros(base, bool)
    for d in fib(100) + GRID:
        px = render_px(d, c, size)
        m = px[:, 3] > 0.5
        q = np.clip(np.round((px[m, :3] * 255 - 4) / 8), 0, 31).astype(np.int64)
        seen[np.unique(q[:, 0] | (q[:, 1] << 5) | (q[:, 2] << 10))] = True
    for t in temps:
        me = t.data; bpy.data.objects.remove(t); bpy.data.meshes.remove(me)
    # per mesh datablock: union over instances, one-face margin, delete the rest
    keep_by_mesh = collections.defaultdict(set)
    for o, b0, n in owner:
        keep_by_mesh[o.data.name] |= set(np.nonzero(seen[b0:b0 + n])[0].tolist())
    report = {}
    for mname, keep in keep_by_mesh.items():
        me = bpy.data.meshes[mname]
        bm = bmesh.new(); bm.from_mesh(me); bm.faces.ensure_lookup_table()
        margin = set(keep)
        for i in keep:
            for e in bm.faces[i].edges:
                for f in e.link_faces:
                    margin.add(f.index)
        kill = [f for f in bm.faces if f.index not in margin]
        n0 = len(bm.faces)
        bmesh.ops.delete(bm, geom=kill, context='FACES')
        bm.to_mesh(me); bm.free(); me.update()
        report[mname] = (n0, len(me.polygons))
    for o in objs:
        o.hide_render = False
    for o in others:
        o.hide_render = False
    return objs, report


def gap_check(objs):
    """Joined copy of the collection, white on black, culled vs unculled from the 26 grid directions."""
    dg = bpy.context.evaluated_depsgraph_get()
    import libcombo
    parts = [(bpy.data.meshes.new_from_object(o.evaluated_get(dg)), o.matrix_world.copy()) for o in objs]
    me = libcombo.join_meshes("__ap_gap", parts)
    for p, _ in parts:
        bpy.data.meshes.remove(p)
    t = bpy.data.objects.new("__ap_gap", me); sc.collection.objects.link(t)
    hidden = [o for o in sc.objects if o.type == 'MESH' and o is not t and not o.hide_render]
    for o in hidden:
        o.hide_render = True
    sh.color_type = 'SINGLE'; sh.single_color = (1, 1, 1)
    c, size = frame([t])
    total = 0
    for d in GRID:
        sh.show_backface_culling = False; a0 = render_px(d, c, size)[:, 3] > 0.5
        sh.show_backface_culling = True; a1 = render_px(d, c, size)[:, 3] > 0.5
        total += int((a0 != a1).sum())
    bpy.data.objects.remove(t); bpy.data.meshes.remove(me)
    for o in hidden:
        o.hide_render = False
    return total


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
summary = {}
for cname in COLLS:
    objs, rep = prune(cname)
    gaps = gap_check(objs)
    dg = bpy.context.evaluated_depsgraph_get(); tris = 0
    for o in objs:
        ev = o.evaluated_get(dg); m = ev.to_mesh(); tris += sum(len(p.vertices) - 2 for p in m.polygons); ev.to_mesh_clear()
    print("PRUNE", cname, "faces kept per mesh", {k: v for k, v in rep.items()}, "tris now", tris, "gap pixels", gaps, flush=True)
    summary[cname] = (tris, gaps)
for c_ in bpy.data.collections:
    if c_.name == "HP":
        bpy.context.view_layer.layer_collection.children["HP"].exclude = True
bpy.data.objects.remove(cam)
if os.path.exists(TMP):
    os.remove(TMP)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
print("PRUNE saved", OUT, summary, flush=True)
