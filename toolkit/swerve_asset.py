# Swerve asset step 4: turn the simplified module pieces into a library collection asset laid out like the MK4i one:
#   <NAME> (empty) -> "<NAME> Parts" / "<NAME> Wheel" (empties) -> one low-poly object per part (origin at its centre,
#   identical parts share a mesh), library finishes from RobotMaterials. NO motors in the asset (owner: "the motors should
#   not be in the swerve asset. this goes for any module you simplify into an asset"): the fitted library-motor poses go
#   to asset_info.json "motor_mounts" (module frame), and the robot places the library motors as separate objects
#   (pipeline cfg.LIB_IDENTITY_MOTORS).
#   The cleaned CAD module goes into an excluded "HP" collection for reference (as in the MK4i file).
#   ROBOT=<config> blender -b swerve_simplified.blend --python swerve_asset.py -- <work dir> <out.blend> <asset name>
import bpy, sys, os, json, math, hashlib, collections
import numpy as np
from mathutils import Vector, Matrix
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import align, shading
D, OUT, NAME = sys.argv[sys.argv.index("--") + 1:][:3]
SHORT = "Swerve mk2"
import paths  # noqa: E402
RM = paths.robot_materials()
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"  # same catalog as the MK4i asset
sel = json.load(open(os.path.join(D, "select.json")))
motors = json.load(open(os.path.join(D, "motors.json")))
sc = bpy.context.scene
bpy.context.preferences.filepaths.save_version = 0


def principled(m):
    if m and m.use_nodes and m.node_tree:
        for n in m.node_tree.nodes:
            if n.bl_idname == 'ShaderNodeBsdfPrincipled':
                return n
    return None


# ---------------------------------------------------------------- finishes: coloured by part type like the MK4i asset
# (the CAD's own colours are wrong, owner). Library materials are left exactly as RobotMaterials ships them.
FINISH = {"Plate": "Aluminum - Machined", "Standoff": "Aluminum - Machined", "Shaft": "Aluminum - Machined",
          "Spacer": "Aluminum - Machined", "Wheel Hub": "Aluminum - Machined", "Wheel Mount": "Aluminum - Machined",
          "Bearing Ring": "Aluminum - Machined", "Top Cap": "Aluminum - Machined", "Part": "Aluminum - Machined",
          "Screw": "Steel - Gear", "Gear": "Steel - Gear", "Bevel Gear": "Steel - Gear",
          "Steering Pulley": "Aluminum - Polished", "Bearing": "Steel - Machined",
          "Wheel Tread": "Rubber - Black"}  # owner: the tread is black like the CAD's, not Blue Nitrile
KIND = {"Swerve CAD~026": "Wheel Tread", "Swerve CAD~062": "Wheel Hub", "Swerve CAD~168": "Wheel Mount",
        "Swerve CAD~169": "Wheel Mount", "Swerve CAD~057": "Bevel Gear", "Swerve CAD~063": "Gear", "Swerve CAD~065": "Bearing",
        "Swerve CAD~066": "Bearing", "Swerve CAD~030": "Bearing Ring", "Swerve CAD~050": "Steering Pulley", "Swerve CAD~165": "Top Cap"}
with bpy.data.libraries.load(RM, link=False) as (src, dst):
    dst.materials = sorted(set(FINISH.values()))
LIBM = {m.name: m for m in dst.materials if m}


def one_finish(me, kind):
    """Single material per part (as on the MK4i asset)."""
    me.materials.clear()
    me.materials.append(LIBM[FINISH[kind]])
    me.polygons.foreach_set("material_index", np.zeros(len(me.polygons), np.int32))


# ---------------------------------------------------------------- piece meshes (Decimate applied, shading kept live)
def baked_mesh(o):
    keep = []
    for m in o.modifiers:
        if m.type == 'WEIGHTED_NORMAL' or (m.type == 'NODES' and m.node_group and m.node_group.name.startswith(shading.SBA)):
            keep.append((m, m.show_viewport, m.show_render))
            m.show_viewport = m.show_render = False
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(o.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
    for m, v, r in keep:
        m.show_viewport, m.show_render = v, r
    return me, bool(keep)


METHOD = {r["object"]: r["method"] for r in json.load(open(os.path.join(D, "report.json")))["rows"]}


def kind_of(n, o_sel, me):
    if n in KIND:
        return KIND[n]
    e = o_sel["ext"]
    if o_sel["cat"] == "screw":
        return "Screw"
    if METHOD.get(n) == "cylinder rebuild":  # lathe: a rod is a standoff/shaft, a disc is a gear/spacer
        if abs(e[0] - e[1]) < 0.08 * e[1] and e[2] > 1.5 * e[1]:
            return "Standoff" if e[1] <= 10 else "Shaft"
        if abs(e[1] - e[2]) < 0.08 * e[2] and e[0] < e[2]:
            return "Gear" if e[2] >= 18 else "Spacer"
    if e[0] < 0.3 * e[2]:
        return "Plate"
    return "Part"


root = bpy.data.objects.new(NAME, None)
grp = {g: bpy.data.objects.new(f"{NAME} {g}", None) for g in ("Parts", "Wheel")}
for g in grp.values():
    g.parent = root
asset_objs = [root] + list(grp.values())
shared = {}
counts = collections.Counter()
made = []


def add_part(me, kind, group, centre, shade):
    """Object with origin at `centre`; identical geometry (after centring) shares one mesh."""
    me.transform(Matrix.Translation(-Vector(centre)))
    co = np.empty(len(me.vertices) * 3); me.vertices.foreach_get("co", co)
    lv = np.empty(len(me.loops), np.int32); me.loops.foreach_get("vertex_index", lv)
    h = hashlib.sha1(np.round(co, 5).tobytes() + lv.tobytes() + "|".join(m.name for m in me.materials if m).encode()).hexdigest()
    if h in shared:
        bpy.data.meshes.remove(me)
        me = shared[h]
    else:
        counts[kind] += 1
        me.name = f"{SHORT} - {kind}.{counts[kind]:03d}"
        shared[h] = me
    counts[kind + " obj"] += 1
    o = bpy.data.objects.new(f"{SHORT} - {kind}.{counts[kind + ' obj']:03d}", me)
    o.location = Vector(centre)
    o.parent = grp[group]
    if shade:
        shading.add_shading(o)
    asset_objs.append(o)
    made.append(o)
    return o


# regular pieces
stacks = collections.defaultdict(list)
for n, s in sorted(sel.items()):
    if not s["include"] or s["cat"] == "motor":
        continue
    o = bpy.data.objects.get(n)
    if o is None:
        continue
    if s["cat"] == "spacer stack":
        stacks[(round(s["ctr"][0]), round(s["ctr"][1]))].append((n, o, s))
        continue
    me, shade = baked_mesh(o)
    kind = kind_of(n, s, me)
    one_finish(me, kind)
    bb = np.array([list(v.co) for v in me.vertices])
    add_part(me, kind, s["group"], (bb.min(0) + bb.max(0)) / 2, shade)
    print(f"PART {n[-3:]} {kind:16s} {FINISH[kind]:22s} {s['group']:6s} {len(me.polygons)} faces", flush=True)


# spacer stacks -> one tube per contiguous stack (the MK4i models a spacer stack as one cylinder)
def tube(name, cx, cy, z0, z1, r_out, r_in, mat):
    import geo
    n = geo.ngon_segments(r_out * 1000)
    verts, faces = [], []
    for z in (z0, z1):
        for r in (r_out, r_in):
            for k in range(n):
                a = 2 * math.pi * k / n
                verts.append((cx + r * math.cos(a), cy + r * math.sin(a), z))
    B0o, B0i, B1o, B1i = 0, n, 2 * n, 3 * n
    for k in range(n):
        k2 = (k + 1) % n
        faces += [(B0o + k, B0o + k2, B1o + k2, B1o + k), (B0i + k2, B0i + k, B1i + k, B1i + k2),
                  (B1o + k, B1o + k2, B1i + k2, B1i + k), (B0o + k2, B0o + k, B0i + k, B0i + k2)]
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.materials.append(mat)
    me.update()
    return me


for (cx, cy), items in stacks.items():
    items.sort(key=lambda t: t[2]["ctr"][2])
    runs, cur = [], []
    for n, o, s in items:
        co = np.array([list(o.matrix_world @ v.co) for v in o.data.vertices])
        z0, z1 = co[:, 2].min(), co[:, 2].max()
        c2 = co[:, :2].mean(0)
        r = np.linalg.norm(co[:, :2] - c2, axis=1)
        piece = (z0, z1, r.max(), r.min(), c2, o.data.materials[0] if o.data.materials else None)
        if cur and z0 - cur[-1][1] > 0.6e-3:
            runs.append(cur); cur = []
        cur.append(piece)
    runs.append(cur)
    for run in runs:
        z0, z1 = run[0][0], run[-1][1]
        ro, ri = max(p[2] for p in run), min(p[3] for p in run)
        c2 = np.mean([p[4] for p in run], axis=0)
        me = tube("spacer", 0, 0, z0, z1, ro, ri, LIBM[FINISH["Spacer"]])
        add_part(me, "Spacer", "Parts", (0, 0, 0), False)
        made[-1].location = Vector((c2[0], c2[1], 0))

# motors stay out of the asset (owner); their fitted library poses (library space -> module frame) are recorded
motor_mounts = [{"piece": piece, "kind": m["kind"], "lib": m["lib"],
                 "matrix": [list(r) for r in align.to_matrix(np.array(m["R"]), np.array(m["t"]))]}
                for piece, m in sorted(motors.items())]

# ---------------------------------------------------------------- assemble the file: asset collection + HP reference
coll = bpy.data.collections.new(NAME)
sc.collection.children.link(coll)
for o in asset_objs:
    coll.objects.link(o)
# everything else (pipeline pieces, review collections, the work CAD) leaves the file; the CAD module is reloaded as HP
for o in list(bpy.data.objects):
    if o not in asset_objs:
        bpy.data.objects.remove(o)
for c in list(bpy.data.collections):
    if c is not coll:
        bpy.data.collections.remove(c)
with bpy.data.libraries.load(os.path.join(D, "swerve_cad.blend"), link=False) as (src, dst):
    dst.objects = ["Swerve CAD"]
hp = bpy.data.collections.new("HP")
sc.collection.children.link(hp)
cad = dst.objects[0]
cad.name = f"{NAME} (CAD, cleaned)"
hp.objects.link(cad)
hp.hide_render = True
bpy.context.view_layer.layer_collection.children["HP"].exclude = True
bpy.ops.outliner.orphans_purge(do_recursive=True)

coll.asset_mark()
coll.asset_data.catalog_id = CATALOG
coll.asset_data.description = (os.environ.get("ASSET_SOURCE", "Custom") + " swerve module, low-poly in the style of the SDS MK4I asset: plates, "
                               "standoffs, drivetrain, wheel and noticeable screws; washers, small bearings and the encoder left out. "
                               "No motors (place the library Kraken X60 drive + X44 steer motors as their own objects; "
                               "mount poses in the module frame are in the build's asset_info.json).")
for t in ["swerve", "drivetrain"] + [t for t in os.environ.get("ASSET_TAGS", "").split(",") if t]:
    coll.asset_data.tags.new(t)
tris = 0
dg = bpy.context.evaluated_depsgraph_get()
for o in coll.all_objects:
    if o.type == 'MESH':
        ev = o.evaluated_get(dg); m = ev.to_mesh(); tris += sum(len(p.vertices) - 2 for p in m.polygons); ev.to_mesh_clear()
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
info = {"objects": len([o for o in coll.all_objects if o.type == 'MESH']), "meshes": len(shared), "tris": tris,
        "kinds": {k: v for k, v in counts.items() if not k.endswith(" obj")},
        "finishes": FINISH, "motor_mounts": motor_mounts}
json.dump(info, open(os.path.join(D, "asset_info.json"), "w"), indent=1)
print("ASSET", json.dumps({k: info[k] for k in ("objects", "meshes", "tris", "kinds")}), flush=True)
