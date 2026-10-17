# Phase 3 (owner's final pass): Limited Dissolve (<= 5 deg, delimit MATERIAL, no boundary dissolve) once more on every
# mesh datablock, once per shared mesh. Objects whose geometry is still a live Collapse already carry a
# "Limited Dissolve" Decimate (Planar) modifier from the pipeline and are skipped here. Shading modifiers
# (Smooth by Angle -> WeightedNormal) are added per part by --shade name,name,... after looking at the renders.
#   ROBOT=<robot> blender -b in.blend --python phase3.py -- out.blend phase3.json [--shade a,b] [--unshade c,d]
import bpy, bmesh, math, sys, os, json, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import shading, re
try:  # per-robot dissolve angles: cfg.LATHE_DISSOLVE_DEG for lathe/hex rebuilds, cfg.DISSOLVE_DEG_PARTS per part
    import robots
    cfg = robots.load()
except BaseException:
    cfg = None
a = sys.argv[sys.argv.index("--") + 1:]
OUT, INFO = a[0], a[1]
shade = a[a.index("--shade") + 1].split(",") if "--shade" in a else []
unshade = a[a.index("--unshade") + 1].split(",") if "--unshade" in a else []
T0 = time.time()
sc = bpy.context.scene
bpy.context.preferences.filepaths.save_version = 0
users = {}
for o in sc.objects:
    if o.type == 'MESH':
        users.setdefault(o.data.name, []).append(o)
res = {}
for mname, os_ in users.items():
    if any(m.type == 'DECIMATE' and m.decimate_type == 'COLLAPSE' for o in os_ for m in o.modifiers):
        continue
    me = bpy.data.meshes[mname]
    before = sum(len(p.vertices) - 2 for p in me.polygons)
    if before == 0:
        continue
    ang = shading.DISSOLVE_DEG
    if cfg is not None:
        # lathe / hex rebuilds keep the pipeline's own (lower) angle instead of being re-dissolved at 5 deg
        if re.search(r"\[(cyl|hex)\](\.\d+)?$", mname):
            ang = getattr(cfg, "LATHE_DISSOLVE_DEG", ang)
        dp = getattr(cfg, "DISSOLVE_DEG_PARTS", {})
        ang = min([dp[cfg.part(o.name)] for o in os_ if cfg.part(o.name) in dp] + [ang])
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(ang), use_dissolve_boundaries=False,
                             verts=bm.verts, edges=bm.edges, delimit={'MATERIAL'})
    bm.to_mesh(me)
    bm.free()
    me.update()
    res[mname] = [before, sum(len(p.vertices) - 2 for p in me.polygons)]
# viewport colour / metallic / roughness from the Principled BSDF for every material that did not come from the CAD
# (library + asset materials; owner's rule). The FBX export reads the Principled node itself, so Unity is unaffected.
cad_mats = set(json.load(open(a[a.index("--cad-mats") + 1]))) if "--cad-mats" in a else set()
synced = 0
for m in bpy.data.materials:
    if m.name in cad_mats or not (m.use_nodes and m.node_tree):
        continue
    pb = next((n for n in m.node_tree.nodes if n.bl_idname == 'ShaderNodeBsdfPrincipled'), None)
    if pb is None:
        continue
    if not pb.inputs['Base Color'].is_linked:
        m.diffuse_color = (*pb.inputs['Base Color'].default_value[:3], 1.0)
    m.metallic = pb.inputs['Metallic'].default_value
    m.roughness = pb.inputs['Roughness'].default_value
    synced += 1
print("PHASE3 viewport-synced materials:", synced, flush=True)
for n in shade:
    o = bpy.data.objects.get(n)
    if o: shading.add_shading(o)
for n in unshade:
    o = bpy.data.objects.get(n)
    if o: shading.remove_shading(o)
json.dump({"dissolved": res, "shade": shade, "unshade": unshade}, open(INFO, "w"), indent=0)
print(f"PHASE3 dissolved {len(res)} meshes: {sum(v[0] for v in res.values()):,} -> {sum(v[1] for v in res.values()):,} tris, {time.time() - T0:.1f}s", flush=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
