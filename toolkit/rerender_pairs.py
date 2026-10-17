# Re-render the AFTER tiles of a few pairs after a patch (same camera, culling and materials as render_pairs.py, which
# names tiles by list position, so a filtered pairs.json would misnumber them) and write their tris_after back into
# pairs.json; then rerun make_sheets.py on the QA dir.
#   ROBOT=<robot> blender -b <patched piece file>.blend --python rerender_pairs.py -- <qa dir> <idx> [<idx> ...]
import bpy, sys, os, json, re
from mathutils import Vector
a = sys.argv[sys.argv.index("--") + 1:]
QA, IDX = a[0], [int(x) for x in a[1:]]
LIST = os.path.join(QA, "pairs.json"); OUTD = os.path.join(QA, "pairs")
entries = json.load(open(LIST))
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'
sc.render.resolution_x = sc.render.resolution_y = 256
sh = sc.display.shading
sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'; sh.show_cavity = False; sh.show_object_outline = False
sh.show_backface_culling = True
sc.display.render_aa = '8'; sc.view_settings.view_transform = 'Standard'
for m in bpy.data.materials:
    if m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled' and not n.inputs['Base Color'].is_linked:
                m.diffuse_color = (*n.inputs['Base Color'].default_value[:3], 1)
    m.metallic = 0.0; m.roughness = 0.5
cam = bpy.data.cameras.new("pc"); co = bpy.data.objects.new("pc", cam); sc.collection.objects.link(co); sc.camera = co
cam.type = 'ORTHO'
meshes = [o for o in sc.objects if o.type == 'MESH']
for c in bpy.data.collections:
    c.hide_render = False
dg = bpy.context.evaluated_depsgraph_get()
d = Vector((1.0, -1.3, 0.9)).normalized()
for i in IDX:
    e = entries[i]
    show = {n if n in sc.objects else re.sub(r"~\d+$", "", n) for n in e["after"]}
    for o in meshes:
        o.hide_render = o.name not in show
    x, y, z, R = e["frame"]
    ctr = Vector((x, y, z))
    cam.ortho_scale = 2.1 * R; co.location = ctr + d * 4 * R; cam.clip_start = 0.0002; cam.clip_end = 10 * R
    co.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = os.path.join(OUTD, f"{i:03d}_after.png")
    bpy.ops.render.render(write_still=True)
    tris = 0
    for n in show:
        o = sc.objects.get(n)
        if o:
            ev = o.evaluated_get(dg); m = ev.to_mesh(); tris += sum(len(q.vertices) - 2 for q in m.polygons); ev.to_mesh_clear()
    print("RERENDER", i, e["part"], e.get("tris_after"), "->", tris, flush=True)
    e["tris_after"] = tris
json.dump(entries, open(LIST, "w"))
