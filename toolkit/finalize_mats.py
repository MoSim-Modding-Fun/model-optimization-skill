# Materials tidy for a delivered robot file: linked library materials made local, identical ".00N" copies merged into
# their base material, viewport colour/metallic/roughness synced from the Principled BSDF (non-CAD materials).
#   blender -b in.blend --python finalize_mats.py -- out.blend cad_mats.json
import bpy, sys, json, re
OUT, CADM = sys.argv[sys.argv.index("--") + 1:][:2]
cad = set(json.load(open(CADM)))
bpy.context.preferences.filepaths.save_version = 0
made_local = 0
for m in list(bpy.data.materials):
    if m.library:
        m.make_local(); made_local += 1


def sig(m):
    pb = next((n for n in m.node_tree.nodes if n.bl_idname == 'ShaderNodeBsdfPrincipled'), None) if m.node_tree else None
    if pb is None:
        return ("vp",) + tuple(round(x, 3) for x in m.diffuse_color)
    return tuple(round(x, 3) for x in pb.inputs['Base Color'].default_value) + (round(pb.inputs['Metallic'].default_value, 3),
                                                                             round(pb.inputs['Roughness'].default_value, 3))


merged = 0
for m in list(bpy.data.materials):
    base = re.sub(r"\.\d{3}$", "", m.name)
    if base != m.name and base not in cad and base in bpy.data.materials:
        b = bpy.data.materials[base]
        if b is not m and sig(b) == sig(m):
            m.user_remap(b); bpy.data.materials.remove(m); merged += 1
synced = 0
for m in bpy.data.materials:
    if m.name in cad or not m.node_tree:
        continue
    pb = next((n for n in m.node_tree.nodes if n.bl_idname == 'ShaderNodeBsdfPrincipled'), None)
    if pb is None:
        continue
    if not pb.inputs['Base Color'].is_linked:
        m.diffuse_color = (*pb.inputs['Base Color'].default_value[:3], 1.0)
    m.metallic = pb.inputs['Metallic'].default_value; m.roughness = pb.inputs['Roughness'].default_value; synced += 1
for lib in list(bpy.data.libraries):
    if not lib.users_id if hasattr(lib, "users_id") else lib.users == 0:
        bpy.data.libraries.remove(lib)
print("MATS made local", made_local, "merged", merged, "synced", synced, "libraries left", [l.filepath for l in bpy.data.libraries], flush=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
