# Copy a CAD part (set of pieces) into its own file, in its own frame (origin = bbox
# centre of the main piece, axes long -> short; FRAME=normals for box-like parts), objects at identity.
#   bl.sh exploded.blend --python extract.py -- <pieces.json | object name> <out.blend> [main piece regex]
# The frame (origin + axes, work frame) is stored on the scene as "src_frame" so placements can be checked later.
import bpy, sys, json, re, os
import numpy as np
from mathutils import Matrix
a = sys.argv[sys.argv.index("--") + 1:]
src, OUT = a[0], a[1]
names = json.load(open(src)) if src.endswith(".json") else [src]
objs = [bpy.data.objects[n] for n in names if n in bpy.data.objects]
dg = bpy.context.evaluated_depsgraph_get()
meshes = {}
for o in objs:
    m = bpy.data.meshes.new_from_object(o.evaluated_get(dg), depsgraph=dg)
    m.transform(o.matrix_world)
    meshes[o.name] = m
co_of = lambda m: np.array([tuple(v.co) for v in m.vertices])
main = next(o for o in objs if re.search(a[2], o.name)) if len(a) > 2 else \
    max(objs, key=lambda o: np.linalg.norm(np.ptp(co_of(meshes[o.name]), axis=0)))
co = co_of(meshes[main.name])
c0 = 0.5 * (co.min(0) + co.max(0))
_, _, Vt = np.linalg.svd(co - co.mean(0), full_matrices=False)
if os.environ.get("FRAME") == "normals":
    m = meshes[main.name]; m.calc_loop_triangles()
    T = np.array([tuple(t.vertices) for t in m.loop_triangles])
    A_, B_, C_ = co[T[:, 0]], co[T[:, 1]], co[T[:, 2]]
    cr = np.cross(B_ - A_, C_ - A_); ar = np.linalg.norm(cr, axis=1); nn = cr / np.maximum(ar[:, None], 1e-15)
    nn[nn[:, 0] < 0] *= -1

    def strongest(mask):
        best = None
        for i in np.flatnonzero(mask)[::max(1, mask.sum() // 3000)]:
            w = ar[mask & (np.abs(nn @ nn[i]) > 0.995)].sum()
            if best is None or w > best[0]:
                best = (w, nn[i])
        sel = mask & (np.abs(nn @ best[1]) > 0.995)
        v = (nn[sel] * np.sign(nn[sel] @ best[1])[:, None] * ar[sel, None]).sum(0)
        return v / np.linalg.norm(v)
    n1 = strongest(np.ones(len(nn), bool))
    n2 = strongest(np.abs(nn @ n1) < 0.05)
    n2 = n2 - n1 * (n1 @ n2); n2 /= np.linalg.norm(n2)
    Vt = np.array([n1, n2, np.cross(n1, n2)])
ext = np.ptp((co - c0) @ Vt.T, axis=0)
R = Vt[np.argsort(-ext)]
if np.linalg.det(R) < 0:
    R[2] *= -1
L = (co - c0) @ R.T
c0 = c0 + (0.5 * (L.min(0) + L.max(0))) @ R
MAIN = main.name
M = np.eye(4); M[:3, :3] = R; M[:3, 3] = -R @ c0          # world -> local
for o in list(bpy.context.scene.objects):
    bpy.data.objects.remove(o)
for c in list(bpy.data.collections):
    bpy.data.collections.remove(c)
hp = bpy.data.collections.new("HP"); bpy.context.scene.collection.children.link(hp)
for n, m in meshes.items():
    m.transform(Matrix(M.tolist()))
    m.name = n
    ob = bpy.data.objects.new(n, m)
    hp.objects.link(ob)
bpy.context.scene["src_frame"] = json.dumps({"origin_m": c0.tolist(), "axes": R.tolist(), "main": MAIN})
bpy.ops.outliner.orphans_purge(do_recursive=True)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
print("EXTRACT", len(meshes), "pieces; main", MAIN, "extent mm", np.round(np.ptp(L, axis=0) * 1000, 2).tolist(), flush=True)
