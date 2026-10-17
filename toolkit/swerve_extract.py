# Swerve asset step 0: a file with one swerve module at identity (its own frame) + the standalone Kraken CAD meshes.
#   SWERVE_KEEP='{"<module object>": "Swerve CAD", "<Kraken X60 object>": "Kraken X60 CAD", ...}' \
#   blender -b "<robot> - cleaned.blend" --python swerve_extract.py -- swerve_cad.blend
# New file holding only one swerve module (identity transform = the module's own local frame) + the standalone
# Kraken X60 / X44 CAD meshes (for embedded-motor search), each at the origin.
import bpy, sys, os, json
out = sys.argv[sys.argv.index("--") + 1]
src = bpy.data.filepath
keep = json.loads(os.environ["SWERVE_KEEP"])  # {"<module object>": "Swerve CAD", "<X60 object>": "Kraken X60 CAD", "<X44 object>": "Kraken X44 CAD"}
sc = bpy.context.scene
for o in list(sc.objects):
    if o.name not in keep:
        bpy.data.objects.remove(o)
for n, new in keep.items():
    o = bpy.data.objects[n]
    o.parent = None
    o.matrix_world.identity()
    o["cad_name"] = n
    o.name = new
bpy.data.objects["Kraken X60 CAD"].location.x = 0.4
bpy.data.objects["Kraken X44 CAD"].location.x = -0.4
bpy.ops.outliner.orphans_purge(do_recursive=True)
bpy.ops.wm.save_as_mainfile(filepath=out, compress=False)
print("EXTRACT", [(o.name, len(o.data.polygons)) for o in sc.objects])
