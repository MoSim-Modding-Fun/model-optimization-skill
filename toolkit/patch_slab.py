# Patch a finished run's pipeline output with remodel.slab_union rebuilds of named piece objects (flat sheets with holes
# whose plate rebuild was rejected, mitred tube ends, flat rings), built from the CAD pieces in the exploded CAD file;
# same objects and transforms, CAD materials, modifiers cleared (flat sheets: no Collapse, no shading stack). Then rerun
# the post steps (module placement, implode, phase3) from the patched file. Add the parts to cfg.BUILDERS /
# FORCE_METHOD as well, so the next full run builds them the same way.
#   ROBOT=<robot> blender -b <run>/simplified_exploded.blend --python patch_slab.py -- <exploded.blend> <out.blend> <obj> [<obj> ...]
import bpy, sys, os
import numpy as np
TK = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TK)
import remodel, build  # noqa: E402
a = sys.argv[sys.argv.index("--") + 1:]
SRC, OUT, NAMES = a[0], a[1], a[2:]
with bpy.data.libraries.load(SRC, link=False) as (src, dst):
    dst.objects = [n for n in NAMES if n in src.objects]
for cad_o in list(dst.objects):
    stem = cad_o.name if cad_o.name in NAMES else cad_o.name.rsplit(".", 1)[0]
    tgt = bpy.data.objects[stem]
    assert np.allclose(np.array(tgt.matrix_basis), np.array(cad_o.matrix_basis), atol=1e-9)
    me = cad_o.data.copy()
    build.weld(me, 1e-6)
    new, info = remodel.slab_union(stem, me)
    assert new is not None and info.get("open_edges", 1) == 0, info
    new.materials.clear()
    for m in cad_o.data.materials:
        new.materials.append(m)
    new.polygons.foreach_set("material_index", np.zeros(len(new.polygons), np.int32)) if len(cad_o.data.materials) == 1 else None
    new.name = cad_o.data.name.rsplit(".", 1)[0] + " [slab]"
    old = tgt.data
    tgt.modifiers.clear()
    tgt.data = new
    print("SLAB", stem, info, "tris", sum(len(p.vertices) - 2 for p in new.polygons), "was", sum(len(p.vertices) - 2 for p in old.polygons),
          "mats", [m.name for m in new.materials], flush=True)
    if old.users == 0:
        bpy.data.meshes.remove(old)
    bpy.data.objects.remove(cad_o)
    bpy.data.meshes.remove(me)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=False)
print("SLAB saved", OUT, flush=True)
