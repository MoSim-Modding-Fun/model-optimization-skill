# Round trip: import the FBX, compare the named object's split normals with the blend's (mean angle, degrees)
import bpy, sys, math
import numpy as np
fbx, name = sys.argv[sys.argv.index("--") + 1:][:2]
src = bpy.data.objects[name]
dg = bpy.context.evaluated_depsgraph_get()
def loopnormals(o):
    me = o.evaluated_get(dg).data
    co = np.array([me.vertices[l.vertex_index].co[:] for l in me.loops])
    n = np.array([cn.vector[:] for cn in me.corner_normals])
    return co, n
co0, n0 = loopnormals(src)
M0 = np.array(src.matrix_world.to_3x3())
bpy.ops.import_scene.fbx(filepath=fbx)
imp = [o for o in bpy.context.selected_objects if o.name.startswith(name.split(".")[0]) and o.type == 'MESH' and o is not src]
o = imp[0]; dg = bpy.context.evaluated_depsgraph_get()
co1, n1 = loopnormals(o)
M1 = np.array(o.matrix_world.to_3x3())
w0 = n0 @ M0.T; w0 /= np.linalg.norm(w0, axis=1)[:, None]
w1 = n1 @ M1.T; w1 /= np.linalg.norm(w1, axis=1)[:, None]
print("FBXN loops", len(n0), len(n1), "has custom", "custom_normal" in o.data.attributes)
if len(n0) == len(n1):
    ang = np.degrees(np.arccos(np.clip((w0 * w1).sum(1), -1, 1)))
    print("FBXN angle deg mean", round(float(ang.mean()), 3), "p99", round(float(np.percentile(ang, 99)), 3), "max", round(float(ang.max()), 2))
