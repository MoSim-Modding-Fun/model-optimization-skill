# Per-part before/after renders with identical framing.
#   before: blender -b orig.blend  --python render_pairs.py -- before pairs.json outdir
#   after:  blender -b out.blend   --python render_pairs.py -- after  pairs.json outdir
# CULL=1 renders the AFTER pass with backface culling, as Unity draws it (flipped faces and slits show as holes);
# CULL=both culls the BEFORE pass too (single-sided sources, where a sheet seen from behind is blank anyway).
import bpy, sys, os, json, re, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robots, build
cfg = robots.load()
from mathutils import Vector
mode, LIST, OUTD = sys.argv[sys.argv.index("--") + 1:][:3]
os.makedirs(OUTD, exist_ok=True)
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'
sc.render.resolution_x = sc.render.resolution_y = 256
sh = sc.display.shading
sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'; sh.show_cavity = False; sh.show_object_outline = False
sh.show_backface_culling = os.environ.get("CULL", "") == "both" or (mode == "after" and os.environ.get("CULL", "") == "1")
sc.display.render_aa = '8'; sc.view_settings.view_transform = 'Standard'
for m in bpy.data.materials:
    if m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled' and not n.inputs['Base Color'].is_linked:
                m.diffuse_color = (*n.inputs['Base Color'].default_value[:3], 1)
    # render-only: the synced metallic 1.0 of the library Aluminum / Steel finishes renders near-black in
    # Workbench STUDIO light (Unity reads only the base colour), which reads as a colour error in the sheets
    m.metallic = 0.0; m.roughness = 0.5
cam = bpy.data.cameras.new("pc"); co = bpy.data.objects.new("pc", cam); sc.collection.objects.link(co); sc.camera = co
cam.type = 'ORTHO'
meshes = [o for o in sc.objects if o.type == 'MESH']
for c in bpy.data.collections:
    c.hide_render = False
if mode == "before":
    # CAD exports often carry broken custom normals (flat plates render crumpled): show the true geometry
    for me in bpy.data.meshes:
        if "custom_normal" in me.attributes:
            me.attributes.remove(me.attributes["custom_normal"])
            me.shade_smooth()
            me.set_sharp_from_angle(angle=math.radians(30))
    asm = []
    for spec in cfg.LIB_ASSEMBLIES:
        pieces = [o for o in meshes if spec["select"](o)]
        groups = spec["groups"](pieces) if "groups" in spec else ([pieces] if pieces else [])
        for g in groups:
            if spec.get("target") == "largest":
                after = build.largest_piece(g).name
            else:
                after = spec["object"]
            asm.append((spec, [o.name for o in g], after))
    in_asm = {n for _, ns, _ in asm for n in ns}
    users = {}
    for o in meshes:
        if o.name not in in_asm:
            users.setdefault(o.data.name, []).append(o)
    entries = []
    for mname, os_ in sorted(users.items()):
        rep = os_[0]
        entries.append({"key": mname, "part": cfg.part(rep.name), "before": [rep.name], "after": [rep.name], "count": len(os_),
                        "tris_before": sum(len(q.vertices) - 2 for q in rep.data.polygons)})
    for prefix in getattr(cfg, "LIB_EMPTY_IDENTITY", {}):  # collection asset at an assembly empty: one pair per empty
        for emp in [o for o in sc.objects if o.type == 'EMPTY' and o.name.startswith(prefix)][:1]:
            ns = [o.name for o in emp.children_recursive if o.type == 'MESH']
            entries.append({"key": "asm:" + emp.name + " [library]", "part": emp.name + " (whole module)", "before": ns,
                            "after": [emp.name + " [library]"], "count": 1,
                            "tris_before": sum(sum(len(q.vertices) - 2 for q in sc.objects[n].data.polygons) for n in ns)})
    for spec, ns, after in asm:
        if ns:
            entries.append({"key": "asm:" + after, "part": spec["label"], "before": ns, "after": [after],
                            "count": 1, "tris_before": sum(sum(len(q.vertices) - 2 for q in sc.objects[n].data.polygons) for n in ns)})
    for e in entries:
        bb = []
        for n in e["before"]:
            o = sc.objects[n]
            bb += [o.matrix_world @ Vector(c) for c in o.bound_box]
        ctr = sum(bb, Vector()) / len(bb)
        R = max((b - ctr).length for b in bb)
        e["frame"] = [ctr.x, ctr.y, ctr.z, R]
    json.dump(entries, open(LIST, "w"))
else:
    entries = json.load(open(LIST))
dg = bpy.context.evaluated_depsgraph_get()
d = Vector((1.0, -1.3, 0.9)).normalized()
for i, e in enumerate(entries):
    show = set(e[mode])
    if mode == "after":  # exploded pieces were joined back into their object by implode.py
        show = {n if n in sc.objects else re.sub(r"~\d+$", "", n) for n in show}
    for o in meshes:
        o.hide_render = o.name not in show
    x, y, z, R = e["frame"]
    ctr = Vector((x, y, z))
    cam.ortho_scale = 2.1 * R; co.location = ctr + d * 4 * R; cam.clip_start = 0.0002; cam.clip_end = 10 * R
    co.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = os.path.join(OUTD, f"{i:03d}_{mode}.png")
    bpy.ops.render.render(write_still=True)
    if mode == "after":
        tris = 0
        for n in show:
            o = sc.objects.get(n)
            if o:
                ev = o.evaluated_get(dg); m = ev.to_mesh(); tris += sum(len(q.vertices) - 2 for q in m.polygons); ev.to_mesh_clear()
        e["tris_after"] = tris
if mode == "after":
    json.dump(entries, open(LIST, "w"))
