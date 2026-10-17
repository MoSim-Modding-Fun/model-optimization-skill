# Parts shown in context (owner request, 2026-10-10: "send them all in context ... body solid, everything else 20%
# opacity"): per entry, the named objects solid in their own colours and every other mesh under a context empty at a set
# opacity, framed on a frame empty, from two opposite isometric directions. EEVEE (Workbench has no per-object alpha).
#   blender -b robot.blend --python context_shots.py -- <spec.json> <out dir> [long side px]
#   spec: [{"id": "O192", "solid": [object names], "context": "<empty name>", "frame": "<empty name>", "alpha": 0.2}, ...]
#   optional: "frame_objects": [names] (frame on these instead of the frame empty), "pad": 1.1 (frame margin),
#   "highlight": [r, g, b] (the solid objects in one vivid colour: a pale part among 20% ghosts was hard to find)
# Renders <out dir>/<id>_<k>.png (k = 1, 2). The file is not saved.
import bpy, sys, os, json, math
import numpy as np
from mathutils import Vector
a = sys.argv[sys.argv.index("--") + 1:]
SPEC, OUT = json.load(open(a[0])), a[1]
PX = int(a[2]) if len(a) > 2 else 1600
os.makedirs(OUT, exist_ok=True)
VIEWS = [Vector((1.0, -1.3, 0.9)).normalized(), Vector((-1.0, 1.3, -0.9)).normalized()]
sc = bpy.context.scene
sc.render.engine = 'BLENDER_EEVEE'
sc.render.film_transparent = False
sc.view_settings.view_transform = 'Standard'
sc.render.image_settings.file_format = 'PNG'
if not sc.world:
    sc.world = bpy.data.worlds.new("ctx")
sc.world.use_nodes = True
bg = sc.world.node_tree.nodes.get("Background")
bg.inputs[0].default_value = (0.36, 0.37, 0.40, 1); bg.inputs[1].default_value = 1.0
for me in bpy.data.meshes:  # CAD custom normals are often broken (as render_pairs.py)
    if "custom_normal" in me.attributes:
        me.attributes.remove(me.attributes["custom_normal"])
        me.shade_smooth()
        me.set_sharp_from_angle(angle=math.radians(30))
sun_d = bpy.data.lights.new("ctx_sun", 'SUN'); sun_d.energy = 3.0
sun = bpy.data.objects.new("ctx_sun", sun_d); sc.collection.objects.link(sun)
cam_d = bpy.data.cameras.new("ctx_cam"); cam_d.type = 'ORTHO'
cam = bpy.data.objects.new("ctx_cam", cam_d); sc.collection.objects.link(cam); sc.camera = cam
meshes = [o for o in sc.objects if o.type == 'MESH']
for c in bpy.data.collections:
    c.hide_render = False
ghost = {}


def ghost_of(m, alpha):
    """A blended copy of material m at the given opacity (cached)."""
    key = (m.name if m else None, alpha)
    if key not in ghost:
        g = (m.copy() if m else bpy.data.materials.new("ctx_ghost"))
        g.name = f"ctx_ghost {key[0]} {alpha}"
        g.use_nodes = True
        bsdf = next((n for n in g.node_tree.nodes if n.bl_idname == 'ShaderNodeBsdfPrincipled'), None)
        if bsdf is None:
            bsdf = g.node_tree.nodes.new('ShaderNodeBsdfPrincipled')
            out = next(n for n in g.node_tree.nodes if n.bl_idname == 'ShaderNodeOutputMaterial')
            g.node_tree.links.new(bsdf.outputs[0], out.inputs[0])
        bsdf.inputs['Alpha'].default_value = alpha
        bsdf.inputs['Metallic'].default_value = 0.0
        g.surface_render_method = 'BLENDED'
        g.use_backface_culling = False
        ghost[key] = g
    return ghost[key]


def solidify(m):
    if m and m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled':
                n.inputs['Metallic'].default_value = 0.0  # render-only, as the other QA renders
    return m


saved = {o.name: [(s.link, s.material) for s in o.material_slots] for o in meshes}
hl_mats = {}


def highlight(rgb):
    key = tuple(rgb)
    if key not in hl_mats:
        m = bpy.data.materials.new("ctx_highlight"); m.use_nodes = True
        b = next(n for n in m.node_tree.nodes if n.bl_idname == 'ShaderNodeBsdfPrincipled')
        b.inputs['Base Color'].default_value = (*rgb, 1); b.inputs['Roughness'].default_value = 0.45
        hl_mats[key] = m
    return hl_mats[key]


def descendants(name):
    root = bpy.data.objects[name]
    return {o.name for o in root.children_recursive if o.type == 'MESH'}


for e in SPEC:
    solid = set(e["solid"]); ctx = descendants(e["context"]) - solid; alpha = e.get("alpha", 0.2)
    for o in meshes:
        o.hide_render = o.name not in solid and o.name not in ctx
        for i, (lk, m) in enumerate(saved[o.name]):
            s = o.material_slots[i]
            if o.name in ctx:
                s.link = 'OBJECT'; s.material = ghost_of(m, alpha)
            elif o.name in solid and e.get("highlight"):
                s.link = 'OBJECT'; s.material = highlight(e["highlight"])
            else:
                s.link = lk
                if lk == 'OBJECT':
                    s.material = m
                solidify(s.material)
    P = []
    for n in (set(e["frame_objects"]) if e.get("frame_objects") else descendants(e["frame"]) | solid):
        o = bpy.data.objects[n]
        P += [np.array(o.matrix_world @ Vector(c)) for c in o.bound_box]
    P = np.array(P)
    for k, d in enumerate([VIEWS[i - 1] for i in e.get("views", [1, 2])], 1):
        q = (-d).to_track_quat('-Z', 'Y'); M = q.to_matrix()
        rt, up = np.array(M.col[0]), np.array(M.col[1])
        x, y = P @ rt, P @ up
        pad = e.get("pad", 1.1)
        w, h = (np.ptp(x)) * pad, (np.ptp(y)) * pad
        if w >= h:
            sc.render.resolution_x, sc.render.resolution_y, cam_d.ortho_scale = PX, max(640, round(PX * h / w)), w
        else:
            sc.render.resolution_x, sc.render.resolution_y, cam_d.ortho_scale = max(640, round(PX * w / h)), PX, h
        c = (x.max() + x.min()) / 2 * rt + (y.max() + y.min()) / 2 * up + (P @ np.array(d)).mean() * np.array(d)
        R = float(np.linalg.norm(P.max(0) - P.min(0)))
        cam.location = Vector(c) + d * 3 * R; cam.rotation_euler = q.to_euler()
        cam_d.clip_start = 1e-4; cam_d.clip_end = 10 * R + 1
        sun.rotation_euler = (-(d + Vector((0, 0, 0.8))).normalized()).to_track_quat('-Z', 'Y').to_euler()
        sc.render.filepath = os.path.join(OUT, f"{e['id']}_{k}.png")
        bpy.ops.render.render(write_still=True)
    print("CTX", e["id"], len(solid), "solid", len(ctx), "context", flush=True)
