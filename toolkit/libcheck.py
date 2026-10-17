# Library swaps vs the CAD they replaced, in each object's LOCAL frame (the pipeline keeps transforms, so the frames
# match; world bboxes of rotated objects read up to sqrt(2) too big). Per object: the per-axis offset of both bbox faces
# in mm (a pure axial shift = both faces moved the same way: a motor sitting off its mount), and the area centroid of
# distinctively coloured faces (red, green, yellow/orange, blue, white, near-black): a breaker button, an LED bar, a lens
# or battery terminals in the wrong place means the box is turned (LIB_FLIP), even when the fit error ties.
#   blender -b "<source>.blend" --python libcheck.py -- dump cad_lib.json '["object", ...]'
#   blender -b <run>/final.blend --python libcheck.py -- check cad_lib.json [report.json]
# The object list for dump: the library-swapped objects from a run's report.json (rows whose method starts with
# "library"); with no list, every mesh object is dumped (slow on a big CAD). check uses report.json to pick the objects,
# else every object in the dump.
import bpy, sys, json
from mathutils import Vector

a = sys.argv[sys.argv.index("--") + 1:]
CLASSES = {
    "red": lambda r, g, b: r > 0.45 and g < 0.3 and b < 0.3,
    "green": lambda r, g, b: g > 0.45 and r < 0.6 and b < 0.45,
    "yellow": lambda r, g, b: r > 0.5 and g > 0.35 and b < 0.25,
    "blue": lambda r, g, b: b > 0.45 and r < 0.3,
    "white": lambda r, g, b: min(r, g, b) > 0.75,
    "black": lambda r, g, b: max(r, g, b) < 0.06,
}


def colour(m):
    if m and m.use_nodes and m.node_tree:
        pb = next((n for n in m.node_tree.nodes if n.bl_idname == 'ShaderNodeBsdfPrincipled'), None)
        if pb and not pb.inputs['Base Color'].is_linked:
            return tuple(pb.inputs['Base Color'].default_value[:3])
    return tuple(m.diffuse_color[:3]) if m else (0.5, 0.5, 0.5)


def facts(o, dg):
    me = o.evaluated_get(dg).to_mesh()
    if not len(me.vertices):
        o.evaluated_get(dg).to_mesh_clear()
        return None
    xs = [v.co for v in me.vertices]
    mn = [min(v[k] for v in xs) for k in range(3)]
    mx = [max(v[k] for v in xs) for k in range(3)]
    acc = {k: [Vector(), 0.0] for k in CLASSES}
    for p in me.polygons:
        c = colour(me.materials[p.material_index] if me.materials else None)
        for k, test in CLASSES.items():
            if test(*c):
                acc[k][0] += p.center * p.area
                acc[k][1] += p.area
    cent = {k: [round(x * 1000, 1) for x in v[0] / v[1]] for k, v in acc.items() if v[1] > 1e-8}
    area = {k: round(v[1] * 1e6, 1) for k, v in acc.items() if v[1] > 1e-8}
    o.evaluated_get(dg).to_mesh_clear()
    return {"mn": [x * 1000 for x in mn], "mx": [x * 1000 for x in mx], "centroids_mm": cent, "area_mm2": area}


dg = bpy.context.evaluated_depsgraph_get()
if a[0] == "dump":
    names = json.loads(a[2]) if len(a) > 2 else [o.name for o in bpy.data.objects if o.type == 'MESH']
    out = {}
    for n in names:
        o = bpy.data.objects.get(n)
        if o is not None and o.type == 'MESH':
            f = facts(o, dg)
            if f:
                out[n] = f
    json.dump(out, open(a[1], "w"), indent=1)
    print("LIBCHECK dumped", len(out))
else:
    cad = json.load(open(a[1]))
    names = list(cad)
    if len(a) > 2:
        rows = json.load(open(a[2])).get("rows", [])
        names = [r["object"] for r in rows if str(r.get("method", "")).startswith("library") and r["object"] in cad]
    for n in names:
        o = bpy.data.objects.get(n)
        if o is None or o.type != 'MESH':
            continue
        f, c = facts(o, dg), cad[n]
        if not f:
            continue
        d = [[round(f["mn"][k] - c["mn"][k], 2), round(f["mx"][k] - c["mx"][k], 2)] for k in range(3)]
        worst = max(abs(x) for p in d for x in p)
        axial = [k for k in range(3) if abs(d[k][0] - d[k][1]) < 0.3 and abs(d[k][0]) > 0.5]
        moved = {k: round((Vector(f["centroids_mm"][k]) - Vector(c["centroids_mm"][k])).length, 1)
                 for k in f["centroids_mm"] if k in c["centroids_mm"]}
        flag = " CHECK" if worst > 2 or any(v > 10 for v in moved.values()) else ""
        print(f"LIBCHECK {n[:44]:44} faces x {d[0]} y {d[1]} z {d[2]} worst {worst}"
              + (f" axial-shift on {['xyz'[k] for k in axial]}" if axial else "")
              + f" | colour centroid moved mm {moved}{flag}")
