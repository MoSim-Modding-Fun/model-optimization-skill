# A wider version of an existing library flex wheel (2026-10-15): straight flex wheels have the same face pattern
# at every width, so the 1 in wide 1.625 in wheel is the library's 0.5 in "30A, Stretch Bore" asset stretched along its
# axis (z). Same topology and tris; the CAD reference collection is replaced by the new robot's CAD wheel (optional).
#   blender -b <source asset .blend> --python flex_widen_asset.py -- <out .blend> "<new asset name>" <scale z> ["<description>"]
import bpy, sys, os
from mathutils import Matrix
a = sys.argv[sys.argv.index("--") + 1:]
OUT, NAME, S = a[0], a[1], float(a[2])
DESC = a[3] if len(a) > 3 else ""
src = next(o for o in bpy.data.objects if o.asset_data)
src.data.transform(Matrix.Diagonal((1, 1, S, 1)))
src.name = src.data.name = NAME
src.asset_data.description = DESC
for c in [c for c in bpy.data.collections if c.name == "CAD reference"]:
    for o in list(c.objects):
        bpy.data.objects.remove(o)
    bpy.data.collections.remove(c)
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
import numpy as np
co = np.array([v.co for v in src.data.vertices])
print("WIDEN", NAME, "tris", sum(len(p.vertices) - 2 for p in src.data.polygons), "extent mm", np.round(np.ptp(co, 0) * 1000, 2).tolist(), flush=True)
bpy.context.preferences.filepaths.save_version = 0
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
