# Chain runs exported as separate link objects -> ONE loop per run (owner rule, 2026-10-14: "chains should be one loop
# instead of links"; after the pipeline). chainloop.build_chain_loop needs the whole run in one mesh: the
# run's link meshes (CAD, kept unchanged by the pipeline: FORCE_METHOD "keep") are joined in world space, the loop is
# built from that, and lands on a new object "<label>" under the run's parent empty (identity world, so the loop's
# world-space vertices stay put). The link objects move to "REVIEW - Replaced by chain loop" (never deleted). The loop
# takes the links' area-majority material and gets no shading stack (walls smooth, cap edges sharp from the builder).
#   blender -b simplified.blend --python chain_join.py -- <runs.json> <out.blend> [<cad runs .blend>]
# runs.json: [{"label": "#25 Chain loop (left)", "objects": [link object names], "parent": "<empty name>"}, ...]
# cad runs .blend (optional): one world-space mesh object "<label> [CAD links]" per run, joined from the CAD file;
# then the loop is built from the CAD links while the robot's link objects (the pipeline's low-poly link rebuild) only
# move to REVIEW, so the review collection doesn't carry ~250k tris of CAD chain per run.
import bpy, bmesh, sys, os, json, collections
from mathutils import Matrix
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import chainloop
a = sys.argv[sys.argv.index("--") + 1:]
RUNS, OUT = json.load(open(a[0])), a[1]
CADRUNS = {}
if len(a) > 2:
    with bpy.data.libraries.load(a[2], link=False) as (src, dst):
        dst.meshes = [n for n in src.meshes if n.endswith(" [CAD links]")]
    CADRUNS = {m.name[:-len(" [CAD links]")]: m for m in dst.meshes if m}
sc = bpy.context.scene
rev = bpy.data.collections.get("REVIEW - Replaced by chain loop") or bpy.data.collections.new("REVIEW - Replaced by chain loop")
if rev.name not in sc.collection.children:
    sc.collection.children.link(rev)
rev.color_tag = 'COLOR_04'
report = []
for run in RUNS:
    obs = [bpy.data.objects[n] for n in run["objects"] if n in bpy.data.objects]
    bm = bmesh.new()
    area = collections.Counter()
    dg = bpy.context.evaluated_depsgraph_get()
    for o in obs:
        ev = o.evaluated_get(dg); m = ev.to_mesh()
        t = m.copy(); t.transform(o.matrix_world)
        for p in t.polygons:
            area[t.materials[p.material_index].name if t.materials else None] += p.area
        bm.from_mesh(t); bpy.data.meshes.remove(t); ev.to_mesh_clear()
    joined = bpy.data.meshes.new(run["label"] + " [links]")
    bm.to_mesh(joined); bm.free()
    if run["label"] in CADRUNS:  # build from the CAD links; the material vote stays with the robot's links
        bpy.data.meshes.remove(joined)
        joined = CADRUNS[run["label"]].copy()
        if not area:
            area[joined.materials[0].name if joined.materials else None] += 1.0
    new, info = chainloop.build_chain_loop(run["label"], joined)
    bpy.data.meshes.remove(joined)
    if new is None:
        print("CHAIN JOIN failed", run["label"], info, flush=True)
        report.append({"label": run["label"], "links": len(obs), **info})
        continue
    mat = area.most_common(1)[0][0]
    new.materials.clear()
    if mat:
        new.materials.append(bpy.data.materials[mat])
    ob = bpy.data.objects.new(run["label"], new)
    par = bpy.data.objects.get(run.get("parent", ""))
    for c in (obs[0].users_collection if obs else [sc.collection]):
        if not c.name.startswith("REVIEW"):
            c.objects.link(ob)
    if not ob.users_collection:
        sc.collection.objects.link(ob)
    if par:
        ob.parent = par
        ob.matrix_parent_inverse = par.matrix_world.inverted()
    ob.matrix_world = Matrix.Identity(4)
    for o in obs:
        for c in list(o.users_collection):
            c.objects.unlink(o)
        rev.objects.link(o)
    tris = sum(len(p.vertices) - 2 for p in new.polygons)
    print("CHAIN JOIN", run["label"], "links", len(obs), "->", tris, "tris", info, flush=True)
    report.append({"label": run["label"], "links": len(obs), "tris": tris, "material": mat, **info})
os.makedirs(os.path.dirname(OUT), exist_ok=True)
json.dump(report, open(os.path.splitext(OUT)[0] + "_chain.json", "w"), indent=1)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
print("CHAIN saved", OUT, flush=True)
