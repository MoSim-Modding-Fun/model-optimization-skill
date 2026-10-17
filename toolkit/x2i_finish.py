# Last step of the WCP Swerve X2i library asset: after asset_prune.py, drop asset objects whose mesh lost every
# face (bearings fully hidden inside the module), sync each finish's viewport colour / metallic / roughness from its
# Principled BSDF (owner rule for library colours), report the asset's tris and save.
#   blender -b pruned.blend --python x2i_finish.py -- out.blend ["WCP Swerve X2i"]
import bpy, sys, os
a = sys.argv[sys.argv.index("--") + 1:]
OUT = a[0]
NAME = a[1] if len(a) > 1 else "WCP Swerve X2i"
coll = bpy.data.collections[NAME]
gone = []
for o in list(coll.all_objects):
    if o.type == 'MESH' and len(o.data.polygons) == 0:
        gone.append(o.get("cad_part", o.name))
        bpy.data.objects.remove(o)
for m in bpy.data.materials:
    p = next((n for n in m.node_tree.nodes if n.bl_idname == 'ShaderNodeBsdfPrincipled'), None) if m.node_tree else None
    if p and not p.inputs['Base Color'].is_linked:
        m.diffuse_color = (*p.inputs['Base Color'].default_value[:3], 1)
        m.metallic = p.inputs['Metallic'].default_value
        m.roughness = p.inputs['Roughness'].default_value
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
dg = bpy.context.evaluated_depsgraph_get()
tris = objs = 0
for o in coll.all_objects:
    if o.type == 'MESH':
        ev = o.evaluated_get(dg); m = ev.to_mesh(); tris += sum(len(p.vertices) - 2 for p in m.polygons); ev.to_mesh_clear()
        objs += 1
print("X2I finish", NAME, "objects", objs, "tris", tris, "dropped empty:", gone, flush=True)
bpy.context.preferences.filepaths.save_version = 0
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
