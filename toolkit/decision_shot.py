# Close-up renders for owner decisions (REVIEW collections hidden), same cameras for every option.
#   ROBOT=<robot> blender -b robot.blend --python decision_shot.py -- <out prefix> '<json {name: [target, dir, dist]}>' [swap.json]
# swap.json (optional): {"object name": ["library blend", "asset name"]} -> that object's mesh is replaced by the asset
# (fitted by keeping the object's transform), e.g. to show the blue compliant wheel instead of the green one.
import bpy, sys, os, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mathutils import Vector
a = sys.argv[sys.argv.index("--") + 1:]
prefix, views = a[0], json.loads(a[1])
swap = json.load(open(a[2])) if len(a) > 2 else {}
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'; sc.render.resolution_x = 1000; sc.render.resolution_y = 800
sh = sc.display.shading; sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'; sh.show_cavity = False; sh.show_object_outline = False
sh.show_backface_culling = True; sc.display.render_aa = '8'; sc.view_settings.view_transform = 'Standard'
for m in bpy.data.materials:
    if m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled' and not n.inputs['Base Color'].is_linked:
                m.diffuse_color = (*n.inputs['Base Color'].default_value[:3], 1)
for me in (bpy.data.meshes if os.environ.get("KEEP_NORMALS") != "1" else []):  # KEEP_NORMALS=1: baked normals are the subject
    if "custom_normal" in me.attributes:
        me.attributes.remove(me.attributes["custom_normal"]); me.shade_smooth(); me.set_sharp_from_angle(angle=math.radians(30))
for c in bpy.data.collections:
    if c.name.startswith("REVIEW"):
        c.hide_render = True
import paths  # noqa: E402
L = paths.components()
import numpy as np, geo, libcombo
from mathutils import Matrix
from mathutils.kdtree import KDTree


def lib_eval(f, asset, frame_of=None):
    """Evaluated library mesh in library world space; frame_of: use THAT asset's placement (colour variants are laid
    out side by side in one library file, with identical local geometry)."""
    names = [asset] + ([frame_of] if frame_of else [])
    with bpy.data.libraries.load(f"{L}/{f}", link=False) as (src, dst):
        dst.objects = names
    lo = dst.objects[0]; sc.collection.objects.link(lo)
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(lo.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
    me.transform((dst.objects[1] if frame_of else lo).matrix_world)
    for x in dst.objects:
        bpy.data.objects.remove(x)
    return me


def fit(src_me, dst_me, iters=30):
    """(R, t) mapping src (library space) onto dst (object space): one-sided fit + ICP on dst vertices."""
    pco, ptri, _ = geo.mesh_arrays(src_me); hco, htri, _ = geo.mesh_arrays(dst_me)
    R, t, _, _ = libcombo.find_embedded(pco, ptri, hco, htri); R, t = np.asarray(R), np.asarray(t)
    kd = KDTree(len(hco))
    for i, v in enumerate(hco): kd.insert(Vector(v), i)
    kd.balance()
    P = pco[np.random.default_rng(0).choice(len(pco), min(2000, len(pco)), replace=False)]
    for _ in range(iters):
        Q = P @ R.T + t
        nn = np.array([kd.find(Vector(q))[0][:] for q in Q]); d = np.linalg.norm(nn - Q, axis=1)
        k = d < max(1e-3, np.percentile(d, 80)); A, B = P[k], nn[k]; ca, cb = A.mean(0), B.mean(0)
        U, S_, Vt = np.linalg.svd((A - ca).T @ (B - cb)); D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
        R = Vt.T @ D @ U.T; t = cb - R @ ca
    return R, t


for on, spec in swap.items():  # spec: [library blend, asset now in the file, asset to show instead]
    f, cur_asset, new_asset = spec
    o = bpy.data.objects[on]
    old = o.data
    R, t = fit(lib_eval(f, cur_asset), old)
    new = lib_eval(f, new_asset, frame_of=cur_asset)
    M = Matrix.Identity(4)
    for i in range(3):
        for j in range(3): M[i][j] = R[i, j]
        M[i][3] = t[i]
    new.transform(M)
    for ob in [x for x in bpy.data.objects if x.data is old]:
        ob.data = new
    for m in new.materials:
        pb = next((n for n in m.node_tree.nodes if n.bl_idname == 'ShaderNodeBsdfPrincipled'), None) if m and m.node_tree else None
        if pb: m.diffuse_color = (*pb.inputs['Base Color'].default_value[:3], 1)
cam = bpy.data.cameras.new("dc"); co = bpy.data.objects.new("dc", cam); sc.collection.objects.link(co); sc.camera = co
cam.lens = 60; cam.clip_start = 0.005; cam.clip_end = 50
for name, (tgt, dirv, dist) in views.items():
    t = Vector(tgt); d = Vector(dirv).normalized()
    co.location = t + d * dist
    co.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = f"{prefix}_{name}.png"
    bpy.ops.render.render(write_still=True)
