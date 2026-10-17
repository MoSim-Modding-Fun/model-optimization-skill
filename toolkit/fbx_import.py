# Import an FBX into an empty .blend with FULL names and save it. Never touches the FBX.
#   blender -b --factory-startup --python fbx_import.py -- in.fbx out.blend
# Blender 5.x accepts 255-byte names, but io_scene_fbx's validate_blend_names still cuts names over 63 bytes to
# "<55 chars>_<sha1[:7]>" (one game model lost 740 of 3,528 object names that way). Unity names each FBX mesh after its
# node, so a cut name also breaks a prefab's mesh references after a round trip.
import bpy, sys
from io_scene_fbx import import_fbx


def full_name(name):
    assert type(name) == bytes and len(name) <= 255, name
    return name.decode("utf-8", "replace")


import_fbx.validate_blend_names = full_name
a = sys.argv[sys.argv.index("--") + 1:]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=a[0])
print("IMPORTED", len(bpy.data.objects), "objects", len(bpy.data.meshes), "meshes", len(bpy.data.materials), "materials",
      "longest name", max((len(o.name.encode()) for o in bpy.data.objects), default=0), "bytes")
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=a[1], compress=True)
