# REV 6" ION omni wheel as a library OBJECT asset "6in ION Omni Wheel (MAXSpline Bore)" (2026-10-15), in the CAD
# mesh's own frame (wheel axis = the mesh's thinnest axis), so a robot swaps it in with cfg.LIB_IDENTITY on the wheel part.
# The vendor body is one fused mesh; split_extract.py splits it by colour into the hub ("Omni 0") and 16 roller shells
# ("Omni 1.NNN"), and the pipeline (robots/romni.py) decimates the hub. The roller shells are open (only their outer
# skin), so the pipeline's lathe picked radial axes and made domes: here each roller is revolved about its true axis,
# the rim tangent through its centre (circle fit of the shell's mid-length section), with the owner's side tiers and the
# lathe remodel. Finishes: nearest RobotMaterials colour (robots recolour to their CAD with LIB_CAD_COLOURS).
#   blender -b --factory-startup --python omni_asset.py -- <omni work dir> <out.blend>
import bpy, bmesh, sys, os, json
import numpy as np
from mathutils import Matrix, Vector
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import shading, build, geo
WD, OUT = sys.argv[sys.argv.index("--") + 1:][:2]
NAME = "6in ION Omni Wheel (MAXSpline Bore)"
import paths  # noqa: E402
RM = paths.robot_materials()
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"
FINISHES = {"Plastic - Black": (0.182, 0.182, 0.182), "Plastic - Grey 50": (0.734, 0.734, 0.734),
            "Plastic - Light Grey": (0.493, 0.493, 0.493), "Plastic - Grey": (0.267, 0.267, 0.267)}
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
bpy.context.preferences.filepaths.save_version = 0
with bpy.data.libraries.load(RM, link=False) as (src, dst):
    dst.materials = list(FINISHES)
LIBM = {m.name: m for m in dst.materials if m}


def finish_for(cad_mat):
    try:
        c = np.array([float(x) for x in cad_mat.name.split("_")[:3]])
    except (ValueError, AttributeError):
        return LIBM["Plastic - Grey 50"]
    return LIBM[min(FINISHES, key=lambda k: np.linalg.norm(np.array(FINISHES[k]) - c))]


def load(path):
    with bpy.data.libraries.load(path, link=False) as (src, dst):
        dst.objects = [n for n in src.objects]
    obs = [o for o in dst.objects if o and o.type == 'MESH']
    for o in obs:
        sc.collection.objects.link(o)
    bpy.context.view_layer.update()
    return obs


def baked(o):
    for m in o.modifiers:
        if m.type == 'WEIGHTED_NORMAL' or (m.type == 'NODES' and m.node_group and m.node_group.name.startswith(shading.SBA)):
            m.show_viewport = m.show_render = False
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(o.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
    me.transform(o.matrix_world)
    return me


simp = load(os.path.join(WD, "omni_simplified.blend"))
cad = load(os.path.join(WD, "omni_work.blend"))
allco = np.array([o.matrix_world @ v.co for o in cad for v in o.data.vertices])
lo, hi = allco.min(0), allco.max(0)
W0 = (lo + hi) / 2
wax = np.zeros(3); wax[int(np.argmin(hi - lo))] = 1.0      # wheel axis = thinnest extent
parts, log = [], []
hub = next(o for o in simp if o.name == "Omni 0")
me = baked(hub)
for i, m in enumerate(me.materials):
    me.materials[i] = finish_for(m)
parts.append(me)
for o in [o for o in cad if o.name.startswith("Omni 1")]:
    rm = o.data.copy(); rm.transform(o.matrix_world)
    co = np.array([v.co for v in rm.vertices])
    c = co.mean(0)
    rad = (c - W0) - np.dot(c - W0, wax) * wax
    rad /= np.linalg.norm(rad)
    d = np.cross(wax, rad); d /= np.linalg.norm(d)
    t = (co - c) @ d
    mid = co[np.abs(t) < 0.25 * np.ptp(t)]
    u, w = geo.perp_basis(d)
    p2 = np.c_[(mid - c) @ u, (mid - c) @ w]
    (cx, cy), r, _dev = geo.fit_circle(p2)
    centre = c + cx * u + cy * w
    centre = centre + d * (0.5 * (t.min() + t.max()))
    new = build.build_revolve("roller", rm, d, centre, keep_bore=False, nseg=geo.ngon_segments(r * 1000), remodel=True)
    if isinstance(new, tuple):
        new = new[0]
    for p_ in new.polygons:
        p_.use_smooth = True
    new.materials.clear(); new.materials.append(finish_for(o.data.materials[1] if len(o.data.materials) > 1 else o.data.materials[0]))
    new.polygons.foreach_set("material_index", np.zeros(len(new.polygons), np.int32))
    log.append((o.name, round(r * 1000, 2), round(np.ptp(t) * 1000, 2), sum(len(p_.vertices) - 2 for p_ in new.polygons)))
    parts.append(new)
    bpy.data.meshes.remove(rm)
bm = bmesh.new(); mats = []
for me_ in parts:
    tmp = bmesh.new(); tmp.from_mesh(me_)
    remap = []
    for m in me_.materials:
        if m not in mats:
            mats.append(m)
        remap.append(mats.index(m))
    for f in tmp.faces:
        f.material_index = remap[f.material_index] if remap else 0
    tmp.to_mesh(me_); tmp.free()
    bm.from_mesh(me_)
out = bpy.data.meshes.new(NAME)
bm.to_mesh(out); bm.free()
for m in mats:
    out.materials.append(m)
ob = bpy.data.objects.new(NAME, out)
coll = bpy.data.collections.new(NAME); sc.collection.children.link(coll); coll.objects.link(ob)
shading.add_shading(ob)
ob.asset_mark()
ob.asset_data.catalog_id = CATALOG
ob.asset_data.description = ("REV 6 in ION omni wheel, MAXSpline bore (built from " + os.environ.get("ASSET_SOURCE", "a robot CAD") + "): the hub decimated with "
                             "the quality loop, 16 rollers as clean lathe barrels about their true axes. Origin and axes = the "
                             "CAD wheel mesh's own frame (wheel axis along its thinnest extent).")
for t_ in ("wheel", "omni", "REV", "ION"):
    ob.asset_data.tags.new(t_)
tris = sum(len(p.vertices) - 2 for p in out.polygons)
print("OMNI asset", NAME, "tris", tris, "hub", sum(len(p.vertices) - 2 for p in parts[0].polygons), "rollers", log[:3], "...", flush=True)
hp = bpy.data.collections.new("HP"); sc.collection.children.link(hp)
hm = bpy.data.meshes.new(NAME + " (CAD)"); bm = bmesh.new()
for o in cad:
    t_ = o.data.copy(); t_.transform(o.matrix_world); bm.from_mesh(t_); bpy.data.meshes.remove(t_)
bm.to_mesh(hm); bm.free()
for o in simp + cad:
    bpy.data.objects.remove(o)
hp.objects.link(bpy.data.objects.new(hm.name, hm))
hp.hide_render = True
bpy.context.view_layer.layer_collection.children["HP"].exclude = True
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
json.dump({"tris": tris, "rollers": log}, open(os.path.splitext(OUT)[0] + "_info.json", "w"), indent=1)
print("SAVED", OUT)
