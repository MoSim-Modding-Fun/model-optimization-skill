# 2" vectored (mecanum-style) intake wheels as library OBJECT assets "2in Vectored Intake Wheel, Right" / ", Left"
# (2026-10-15). Each asset = a plain 24-sided hub (owner decision; VEC_GROOVED_HUB=1 keeps the pipeline's
# decimated grooved hub) + 6 solid convex roller barrels fitted to the CAD rollers (16-sided, the owner's tier for
# r 6.5 mm; the CAD's axle bores dropped: close-up detail), joined into one mesh in the hub's own mesh frame (so a
# robot swaps it in with cfg.LIB_IDENTITY on the hub part and LIB_ABSORB's the rollers). Finishes: nearest RobotMaterials
# colour (robots recolour to their CAD with LIB_CAD_COLOURS). HP collection = the CAD wheels (excluded).
#   blender -b --factory-startup --python vectored_asset.py -- <mec work dir> <out.blend>
# work dir: mec_work.blend (_work/mec/extract.py: right wheel in its hub frame, left wheel shifted +120 mm in x),
# mec_simplified.blend (+ report.json) from the pipeline run.
import bpy, bmesh, sys, os, json, math
import numpy as np
from mathutils import Matrix, Vector
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import shading
WD, OUT = sys.argv[sys.argv.index("--") + 1:][:2]
import paths  # noqa: E402
RM = paths.robot_materials()
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"
SIDES = 16
OFFSET = {"Right": Vector((0, 0, 0)), "Left": Vector((0.12, 0, 0))}
FINISHES = {"Plastic - CANYellow": (0.991, 0.914, 0.0), "Plastic - Grey 50": (0.734, 0.734, 0.734),
            "Plastic - Light Grey": (0.493, 0.493, 0.493), "Plastic - Black": (0.182, 0.182, 0.182),
            "Plastic - White": (0.827, 0.8, 0.765), "REV - Orange": (1.0, 0.714, 0.349)}
FINISHES = {k: tuple(v) for k, v in json.loads(os.environ["VEC_FINISHES"]).items()} if "VEC_FINISHES" in os.environ else FINISHES

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


# owner (2026-10-15): "the rollers on the vectored wheels should be convex and rounded a little, the hubs can be
# plain". Roller = barrel: CAD radius at mid-length, BARREL_END x that 1.6 mm from each end, the end edge rounded to
# BARREL_CAP x it; every band-to-band bend < 30 deg so Smooth by Angle keeps the barrel smooth, flat end caps crisp.
BARREL_END, BARREL_CAP, ROUND_MM = 0.94, 0.83, 1.6
PLAIN_HUB = os.environ.get("VEC_GROOVED_HUB") != "1"   # owner: plain hub (24-sided solid cylinder over the hub extent)


def roller(o, shift):
    """Solid SIDES-gon barrel fitted to a CAD roller (world = hub frame + shift): PCA long axis, mid radius = 98th
    percentile of the radial distance, length = extent along the axis."""
    co = np.array([o.matrix_world @ v.co for v in o.data.vertices]) - np.array(shift)
    c0 = co.mean(0)
    w, V = np.linalg.eigh(np.cov((co - c0).T))
    ax = V[:, 2]
    t = (co - c0) @ ax
    radial = (co - c0) - np.outer(t, ax)
    centre_r = radial.mean(0)                          # symmetric points: ~0, kept for safety
    r = float(np.percentile(np.linalg.norm(radial - centre_r, axis=1), 98))
    t0, t1 = float(t.min()), float(t.max())
    base = c0 + centre_r
    u = np.cross(ax, [1, 0, 0] if abs(ax[0]) < 0.9 else [0, 1, 0]); u /= np.linalg.norm(u); v = np.cross(ax, u)
    rd = ROUND_MM * 1e-3
    prof = [(t0, r * BARREL_CAP), (t0 + rd, r * BARREL_END), ((t0 + t1) / 2, r), (t1 - rd, r * BARREL_END), (t1, r * BARREL_CAP)]
    bm = bmesh.new()
    rings = [[bm.verts.new(tuple(base + ax * tt + rr * (math.cos(2 * math.pi * k / SIDES) * u + math.sin(2 * math.pi * k / SIDES) * v)))
              for k in range(SIDES)] for tt, rr in prof]
    for a_, b_ in zip(rings[:-1], rings[1:]):
        for k in range(SIDES):
            f = bm.faces.new((a_[k], a_[(k + 1) % SIDES], b_[(k + 1) % SIDES], b_[k]))
            f.smooth = True
    bm.faces.new(list(reversed(rings[0])))
    bm.faces.new(rings[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(o.name + " [roller]")
    bm.to_mesh(me); bm.free()
    me.materials.append(finish_for(o.data.materials[0] if o.data.materials else None))
    return me, r * 1000, (t1 - t0) * 1000


# owner (2026-10-15): "i don't want stuff phasing on the side on the vector wheels it looks bad". The fitted barrels
# reach 0.5-0.9 mm past the hub's end faces (the CAD rollers do too on one side): clip every roller flat at the hub's
# end planes, CLIP_INSET_MM inside them so the roller ends never share a plane with the hub caps (z-fighting).
CLIP_INSET_MM = 0.25


def clip_x(me, lo, hi):
    """Cut a closed convex mesh at the planes x = lo and x = hi, drop what is outside, cap the cuts flat."""
    bm = bmesh.new(); bm.from_mesh(me)
    for x0, n in ((lo, (-1.0, 0.0, 0.0)), (hi, (1.0, 0.0, 0.0))):
        r = bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], plane_co=(x0, 0.0, 0.0),
                                   plane_no=n, clear_outer=True)
        cut = [e for e in r["geom_cut"] if isinstance(e, bmesh.types.BMEdge) and e.is_valid and e.is_boundary]
        if cut:
            for f in bmesh.ops.holes_fill(bm, edges=cut, sides=0)["faces"]:
                f.smooth = False
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me); bm.free()


def plain_hub(hub_me, n=24):
    """Plain solid n-sided cylinder over the hub's extent (wheel axis = x of the hub mesh frame), the hub's material."""
    co = np.array([v.co for v in hub_me.vertices])
    lo, hi = co.min(0), co.max(0)
    cy, cz = (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2
    r = max(hi[1] - lo[1], hi[2] - lo[2]) / 2
    bm = bmesh.new()
    rings = [[bm.verts.new((x, cy + r * math.cos(2 * math.pi * k / n), cz + r * math.sin(2 * math.pi * k / n))) for k in range(n)]
             for x in (lo[0], hi[0])]
    for k in range(n):
        f = bm.faces.new((rings[0][k], rings[1][k], rings[1][(k + 1) % n], rings[0][(k + 1) % n])); f.smooth = True
    bm.faces.new(list(reversed(rings[0]))); bm.faces.new(rings[1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(hub_me.name + " [plain]")
    bm.to_mesh(me); bm.free()
    area = {}
    for p in hub_me.polygons:
        area[p.material_index] = area.get(p.material_index, 0) + p.area
    me.materials.append(hub_me.materials[max(area, key=area.get)] if hub_me.materials else None)
    return me


simp = load(os.path.join(WD, "mec_simplified.blend"))
cad = load(os.path.join(WD, "mec_work.blend"))
info = {}
assets = []
for hand in ("Right", "Left"):
    name = f"2in Vectored Intake Wheel, {hand}"
    hub_s = next(o for o in simp if o.name.startswith(f"{hand} Hub"))
    hub_c = next(o for o in cad if o.name.startswith(f"{hand} Hub"))
    parts = []
    me = baked(hub_s); me.transform(Matrix.Translation(-OFFSET[hand]))
    for i, m in enumerate(me.materials):
        me.materials[i] = finish_for(m)
    if PLAIN_HUB:
        me = plain_hub(me)
    parts.append(me)
    hx = [v.co.x for v in me.vertices]
    lo_x, hi_x = min(hx) + CLIP_INSET_MM * 1e-3, max(hx) - CLIP_INSET_MM * 1e-3
    rl = []
    for o in [o for o in cad if o.name.startswith(f"{hand} Roller")]:
        rm, r_mm, l_mm = roller(o, OFFSET[hand])
        clip_x(rm, lo_x, hi_x)
        parts.append(rm); rl.append((round(r_mm, 2), round(l_mm, 2)))
    # join into one mesh
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
    out = bpy.data.meshes.new(name)
    bm.to_mesh(out); bm.free()
    for m in mats:
        out.materials.append(m)
    ob = bpy.data.objects.new(name, out)
    coll = bpy.data.collections.new(name)
    sc.collection.children.link(coll); coll.objects.link(ob)
    shading.add_shading(ob)
    ob.asset_mark()
    ob.asset_data.catalog_id = CATALOG
    ob.asset_data.description = (f"2 in OD x 0.91 in wide vectored (mecanum-style) intake wheel, {hand.lower()}-handed, 1/2 in hex "
                                 "bore (built from " + os.environ.get("ASSET_SOURCE", "a robot CAD") + "): plain 24-sided hub (owner), 6 rollers as solid "
                                 f"{SIDES}-sided convex barrels with rounded ends (owner), cut flat just inside the hub's sides "
                                 "(owner: nothing poking out of the sides). Origin and axes = the CAD hub's own mesh "
                                 "frame (wheel axis along x).")
    for t_ in ("wheel", "vectored", "mecanum", "intake"):
        ob.asset_data.tags.new(t_)
    tris = sum(len(p.vertices) - 2 for p in out.polygons)
    info[name] = {"tris": tris, "hub_tris": sum(len(p.vertices) - 2 for p in parts[0].polygons), "rollers_r_len_mm": rl,
                  "materials": [m.name for m in mats]}
    print("VECTORED", name, json.dumps(info[name]), flush=True)
    assets.append(ob)
    # HP: CAD wheel joined (hub frame), excluded later
    cm = [o for o in cad if o.name.startswith((f"{hand} Hub", f"{hand} Roller"))]
    hp = bpy.data.meshes.new(name + " (CAD)")
    bm = bmesh.new()
    for o in cm:
        t_ = o.data.copy(); t_.transform(Matrix.Translation(-OFFSET[hand]) @ o.matrix_world); bm.from_mesh(t_); bpy.data.meshes.remove(t_)
    bm.to_mesh(hp); bm.free()
    info[name]["hp_mesh"] = hp.name
for o in simp + cad:
    bpy.data.objects.remove(o)
hpc = bpy.data.collections.new("HP"); sc.collection.children.link(hpc)
for name in info:
    hpc.objects.link(bpy.data.objects.new(name + " (CAD)", bpy.data.meshes[info[name]["hp_mesh"]]))
hpc.hide_render = True
bpy.context.view_layer.layer_collection.children["HP"].exclude = True
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
json.dump(info, open(os.path.splitext(OUT)[0] + "_info.json", "w"), indent=1)
print("SAVED", OUT)
