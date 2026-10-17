# Isolated renders of named objects (evaluated, local frame, world scale), same cameras for every file.
#   blender -b file.blend --python part_shots.py -- <out dir> <tag> '<json {name: {"dirs": [[x,y,z],..], "dist": [k,..]} or [such dicts]}>'
#   [optional 4th json: {object: {"collapse": ratio}}]
#   (a json argument starting with @ is read from that file)
#   optional "target": [x,y,z] (local) + "radius": framing radius in m
#   dist k: camera distance = k * bounding radius (fit), or "<m>m" = absolute metres (e.g. "1.2m" = about 4 ft)
import bpy, sys, os, json, math
import numpy as np
from mathutils import Vector, Matrix
a = sys.argv[sys.argv.index("--") + 1:]
OUT, TAG = a[0], a[1]
spec = json.load(open(a[2][1:])) if a[2].startswith("@") else json.loads(a[2])
os.makedirs(OUT, exist_ok=True)
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'; sc.render.resolution_x = 900; sc.render.resolution_y = 700
sh = sc.display.shading; sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'; sh.show_cavity = False; sh.show_object_outline = False
sh.show_backface_culling = True; sh.show_specular_highlight = True; sc.display.render_aa = '8'; sc.view_settings.view_transform = 'Standard'
if sc.world:  # asset files can have no world
    sc.world.color = (0.18, 0.18, 0.2)
for m in bpy.data.materials:
    if m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled' and not n.inputs['Base Color'].is_linked:
                m.diffuse_color = (*n.inputs['Base Color'].default_value[:3], 1)
                m.metallic = 0.0; m.roughness = 0.5
# optional 4th argument: {object: {"collapse": ratio}} applied before evaluating (ratio sweeps without saving)
for on, tw in (json.loads(a[3]) if len(a) > 3 else {}).items():
    for m in bpy.data.objects[on].modifiers:
        if m.type == 'DECIMATE' and m.decimate_type == 'COLLAPSE' and "collapse" in tw:
            m.ratio = tw["collapse"]
dg = bpy.context.evaluated_depsgraph_get()
keep = {}
for name in spec:
    o = bpy.data.objects[name]
    me = bpy.data.meshes.new_from_object(o.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
    keep[name] = (me, [s.material for s in o.material_slots])
for o in list(bpy.data.objects):
    bpy.data.objects.remove(o)
cam_d = bpy.data.cameras.new("cam"); cam_d.lens = 50; cam_d.clip_start = 0.005
cam = bpy.data.objects.new("cam", cam_d); sc.collection.objects.link(cam); sc.camera = cam
for name, specs in spec.items():
    me, mats = keep[name]
    o = bpy.data.objects.new("show", me); sc.collection.objects.link(o)
    co = np.array([v.co for v in me.vertices]); c0 = (co.max(0) + co.min(0)) / 2; r0 = np.linalg.norm(co.max(0) - co.min(0)) / 2
    ntri = sum(len(p.vertices) - 2 for p in me.polygons)
    for s in (specs if isinstance(specs, list) else [specs]):
        c, r = c0, r0
        if "target" in s:  # close-up on a point (object local coordinates), dist then absolute or k * "radius" r
            c = np.array(s["target"]); r = s.get("radius", r)
        for i, d in enumerate(s["dirs"]):
            d = Vector(d).normalized()
            for k in s.get("dist", [2.6]):
                dist = float(k[:-1]) if isinstance(k, str) else k * r
                cam.location = Vector(c) + d * dist
                cam.rotation_euler = (-d).to_track_quat('-Z', 'Y' if abs(d.z) < 0.9 else 'X').to_euler()
                lab = s.get('label', name.split('-')[3] if name.count('-') > 3 else name)
                sc.render.filepath = os.path.join(OUT, f"{lab}_{TAG}_v{i}_{k}.png".replace(" ", "_"))
                bpy.ops.render.render(write_still=True)
    print("SHOT", name, TAG, "tris", ntri, "r mm", round(r * 1000, 1), flush=True)
    bpy.data.objects.remove(o)
