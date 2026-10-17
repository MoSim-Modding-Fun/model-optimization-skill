# One render per unique part and view, CAD and simplified with the same tight orthographic framing (owner request,
# 2026-10-10: "images of each unique object, at least 2, one object per image, each a unique id").
# Reads render_pairs.py's pairs.json (+ make_sheets.py's pairs_order.json for the id order) of a QA dir.
#   before: blender -b orig.blend --python object_shots.py -- before <qa dir> <out dir> [long side px]
#   after:  blender -b out.blend  --python object_shots.py -- after  <qa dir> <out dir>
#   then:   python3 object_compose.py <out dir> <final dir>
# The before pass fits the frames (all vertices of the CAD part projected per view) and writes <out dir>/shots.json;
# the after pass reuses them. Views: two opposite isometric directions (every face seen at least obliquely).
import bpy, sys, os, json, re, math
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robots
cfg = robots.load()
from mathutils import Vector
a = sys.argv[sys.argv.index("--") + 1:]
mode, QA, OUT = a[0], a[1], a[2]
PX = int(a[3]) if len(a) > 3 else 1600
MIN_PX = 640
MM_PER_PX, MAX_PX = 0.15e-3, 3200  # long parts get more pixels (a 4 mm channel hole stays ~27 px wide), up to 3200
PAD = 1.08
VIEWS = [Vector((1.0, -1.3, 0.9)).normalized(), Vector((-1.0, 1.3, -0.9)).normalized()]
os.makedirs(os.path.join(OUT, "renders"), exist_ok=True)
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'; sc.render.film_transparent = True
sh = sc.display.shading
sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'; sh.show_cavity = False; sh.show_object_outline = False
sh.show_specular_highlight = True
sh.show_backface_culling = mode == "after"  # the game look (Unity culls back faces); CAD normals are unreliable
sc.display.render_aa = '8'; sc.view_settings.view_transform = 'Standard'
for m in bpy.data.materials:
    if m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled' and not n.inputs['Base Color'].is_linked:
                m.diffuse_color = (*n.inputs['Base Color'].default_value[:3], 1)
    m.metallic = 0.0; m.roughness = 0.5  # render-only, as render_pairs.py (metallic 1.0 renders near-black)
cam = bpy.data.cameras.new("os"); co = bpy.data.objects.new("os", cam); sc.collection.objects.link(co); sc.camera = co
cam.type = 'ORTHO'
meshes = [o for o in sc.objects if o.type == 'MESH']
for c in bpy.data.collections:
    c.hide_render = False
order = json.load(open(os.path.join(QA, "pairs_order.json")))
SJ = os.path.join(OUT, "shots.json")


def basis(d):
    q = (-d).to_track_quat('-Z', 'Y')
    m = q.to_matrix()
    return q, np.array(m.col[0]), np.array(m.col[1])  # camera right, up


if mode == "before":
    for me in bpy.data.meshes:  # as render_pairs.py: broken CAD custom normals crumple flat faces
        if "custom_normal" in me.attributes:
            me.attributes.remove(me.attributes["custom_normal"])
            me.shade_smooth()
            me.set_sharp_from_angle(angle=math.radians(30))
    shots = []
    for i, e in enumerate(order):
        P = []
        for n in e["before"]:
            o = sc.objects[n]
            v = np.empty(len(o.data.vertices) * 3); o.data.vertices.foreach_get("co", v); v = v.reshape(-1, 3)
            M = np.array(o.matrix_world)
            P.append(v @ M[:3, :3].T + M[:3, 3])
        P = np.concatenate(P)
        frames = []
        for d in VIEWS:
            _, rt, up = basis(d)
            x, y = P @ rt, P @ up
            w, h = (x.max() - x.min()) * PAD, (y.max() - y.min()) * PAD
            w, h = max(w, 1e-4), max(h, 1e-4)
            L = int(min(MAX_PX, max(PX, max(w, h) / MM_PER_PX)))
            if w >= h:
                rx, ry, osc = L, max(MIN_PX, round(L * h / w)), w
            else:
                rx, ry, osc = max(MIN_PX, round(L * w / h)), L, h
            c = (x.max() + x.min()) / 2 * rt + (y.max() + y.min()) / 2 * up + (P @ np.array(d)).mean() * np.array(d)
            R = float(np.linalg.norm(P.max(0) - P.min(0)))
            frames.append({"c": c.tolist(), "res": [rx, ry], "ortho": float(osc), "R": R})
        shots.append({"i": i, "key": e["key"], "frames": frames})
    json.dump(shots, open(SJ, "w"))
else:
    shots = json.load(open(SJ))
dg = bpy.context.evaluated_depsgraph_get()
meta = []
for s, e in zip(shots, order):
    assert s["key"] == e["key"]
    show = set(e[mode])
    if mode == "after":  # exploded pieces were joined back into their object by implode.py
        show = {n if n in sc.objects else re.sub(r"~\d+$", "", n) for n in show}
    for o in meshes:
        o.hide_render = o.name not in show
    for k, (d, f) in enumerate(zip(VIEWS, s["frames"])):
        q, _, _ = basis(d)
        sc.render.resolution_x, sc.render.resolution_y = f["res"]
        cam.ortho_scale = f["ortho"]; cam.clip_start = 1e-4; cam.clip_end = 10 * f["R"] + 1
        co.location = Vector(f["c"]) + d * 3 * f["R"]; co.rotation_euler = q.to_euler()
        sc.render.filepath = os.path.join(OUT, "renders", f"{s['i']:03d}_{mode}_{k + 1}.png")
        bpy.ops.render.render(write_still=True)
    if mode == "after":
        tris, review, asset, copies = 0, set(), None, e["count"]
        for n in show:
            o = sc.objects.get(n)
            if not o:
                continue
            ev = o.evaluated_get(dg); m = ev.to_mesh(); tris += sum(len(p.vertices) - 2 for p in m.polygons); ev.to_mesh_clear()
            review |= {c.name for c in o.users_collection if c.name.startswith("REVIEW")}
            if n.endswith("[library]"):  # one pair per module asset: count its placed copies (they share the mesh)
                copies = sum(1 for x in meshes if x.data == o.data)
                for pre, spec in getattr(cfg, "LIB_EMPTY_IDENTITY", {}).items():
                    if n.startswith(pre):
                        asset = spec[1]
            spec = getattr(cfg, "LIB_IDENTITY", {}).get(cfg.part(n))  # as pipeline.py looks it up
            if spec:
                asset = spec[1]
        meta.append({"i": s["i"], "key": e["key"], "part": e["part"], "method": e["method"], "count": copies,
                     "tris_before": e["tris_before"], "tris_after": tris, "review": sorted(review), "asset": asset,
                     "objects": sorted(show)})
    print("SHOT", s["i"], mode, e["part"], flush=True)
if mode == "after":
    json.dump(meta, open(os.path.join(OUT, "meta.json"), "w"), indent=1)
