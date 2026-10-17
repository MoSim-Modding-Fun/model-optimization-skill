# Build a .blend from step_import.py's export:
#   blender -b --factory-startup --python step_build.py -- <export dir> <out.blend> [root fix json]
# Layout (what the pipeline expects from the Onshape glTF imports, see README "Per-robot config"):
#   root empty (top product name) -> one empty per assembly occurrence (its STEP instance name, e.g. "Hopper:1")
#   -> one MESH object per part occurrence "<product>__<k>" (k = 1-based count of that product), or, for a part with
#   several solids, an empty "<product>__<k>" with one mesh child per solid "<product>__<k>~<solid>".
#   cfg.part(): strip "~<n>", then name.split("__")[0] -> the product name.
# Every instance of a solid shares one mesh datablock ("m<part>s<solid>[c<style>]", deterministic for the same STEP).
# Meshes are welded at 1 um (OCC tessellates face by face), shaded smooth with sharp edges above 30 deg, no custom
# normals. Materials: one per CAD colour, named "r_g_b_a" (sRGB as in the STEP), base + viewport colour set.
# The optional root fix json {"matrix": 4x4} is applied to the root empty (up axis / floor), after which every object
# keeps its parent-relative transform.
import bpy, bmesh, json, math, os, re, sys
import numpy as np

bpy.ops.wm.read_factory_settings(use_empty=True)  # no default cube / camera / light

args = sys.argv[sys.argv.index("--") + 1:]
SRC, OUT = args[0], args[1]
FIX = json.load(open(args[2]))["matrix"] if len(args) > 2 else None

tree = json.load(open(os.path.join(SRC, "tree.json")))
npz = np.load(os.path.join(SRC, "meshes.npz"))


def lin(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


mats = []
for r, g, b, a in tree["materials"]:
    m = bpy.data.materials.new(f"{r:.6f}_{g:.6f}_{b:.6f}_{a:.6f}")
    col = (lin(r), lin(g), lin(b), a)
    m.diffuse_color = col
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value = col
        bsdf.inputs["Roughness"].default_value = 1.0
        if a < 1:
            bsdf.inputs["Alpha"].default_value = a
    m.roughness = 1.0
    mats.append(m)

meshes = {}
geom = {}  # geometry hash -> mesh: copies of one component under other product names (Fusion "(1)", "v2") share a mesh


def get_mesh(name):
    if name in meshes:
        return meshes[name]
    V = npz[name + "_v"].astype(np.float64)
    T = npz[name + "_t"]
    M = npz[name + "_m"].astype(np.int64)
    key = (len(V), len(T), hash(np.round(V, 6).tobytes()), hash(T.tobytes()), hash(M.tobytes()))
    if key in geom:
        meshes[name] = geom[key]
        return geom[key]
    me = bpy.data.meshes.new(name)
    me.vertices.add(len(V))
    me.vertices.foreach_set("co", V.ravel())
    me.loops.add(T.size)
    me.loops.foreach_set("vertex_index", T.ravel())
    me.polygons.add(len(T))
    me.polygons.foreach_set("loop_start", np.arange(0, T.size, 3))
    used = sorted(set(M.tolist()))
    remap = {m: i for i, m in enumerate(used)}
    for m in used:
        me.materials.append(mats[m])
    me.polygons.foreach_set("material_index", np.array([remap[m] for m in M.tolist()]))
    me.update()
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)
    bmesh.ops.dissolve_degenerate(bm, edges=bm.edges, dist=1e-7)
    bm.to_mesh(me)
    bm.free()
    me.shade_smooth()
    me.set_sharp_from_angle(angle=math.radians(30))
    me.update()
    meshes[name] = geom[key] = me
    return me


col = bpy.data.collections.new("CAD")
bpy.context.scene.collection.children.link(col)
used_names = set()


def uniq(n):
    assert len(n.encode()) <= 63, n
    if n not in used_names and n not in bpy.data.objects:
        used_names.add(n)
        return n
    k = 2
    while f"{n[:57]} <{k}>" in used_names or f"{n[:57]} <{k}>" in bpy.data.objects:
        k += 1
    n = f"{n[:57]} <{k}>"
    used_names.add(n)
    return n


# Blender names are <= 63 bytes: shorten the long fastener names (the keywords the pipeline matches survive: BHCS / SHCS
# are lathe keywords), then truncate; two products that would share a shortened name get "#2", "#3" ...
ABBREV = [("Hexagon Socket Button Head Cap Screw", "BHCS"), ("Hexagon Socket Head Cap Screw", "SHCS"),
          ("18-8 Stainless Steel", "18-8 SS"), ("316 Stainless Steel", "316 SS"), (" ASME B18.3 -", ""), (" ANSI B18.3 -", ""), (" ASME B18.7 -", ""), (" ASME B18.2.1 -", "")]
base_of, bases = {}, set()


def short(prod):
    if prod in base_of:
        return base_of[prod]
    b = prod
    for a, r in ABBREV:
        b = b.replace(a, r)
    b = re.sub(r"\s+", " ", b).strip()[:52].rstrip()
    k, bb = 2, b
    while bb in bases:
        bb = f"{b[:49].rstrip()}#{k}"
        k += 1
    bases.add(bb)
    base_of[prod] = bb
    return bb


from mathutils import Matrix
count = {}
objs = []
for i, nd in enumerate(tree["nodes"]):
    parent = objs[nd["parent"]] if nd["parent"] >= 0 else None
    M = Matrix(nd["local"])
    prod = (nd["prod"] or nd["inst"] or f"node{i}").strip()
    # unnamed products (OCC names them by shape type) and Fusion's default body / component names say nothing: name
    # them after the assembly they sit in ("Large Roller", "Bumpers - Body1 (1)")
    pprod = tree["nodes"][nd["parent"]]["prod"].strip() if nd["parent"] >= 0 else ""
    if pprod and re.fullmatch(r"(COMPOUND|SOLID|SHELL|COMPSOLID)", prod):
        prod = pprod
    elif pprod and re.fullmatch(r"(Body|Component)\d*( \(\d+\))?(\(Mirror\))?", prod):
        prod = f"{pprod} - {prod}"
    if nd["asm"] or not nd.get("solids"):
        name = prod if parent is None else (nd["inst"] or prod).strip()
        if len(name.encode()) > 57:  # long instance name "<product>:<n>": shorten the product part
            head, sep, tail = name.rpartition(":")
            if not sep:
                head, tail = name, ""
            for a, r in ABBREV:
                head = head.replace(a, r)
            name = head[:56 - len(tail) - len(sep)].rstrip() + sep + tail
        name = uniq(name)
        ob = bpy.data.objects.new(name, None)
        ob.empty_display_size = 0.02
    else:
        count[prod] = count.get(prod, 0) + 1
        base = f"{short(prod)}__{count[prod]}"
        sol = nd["solids"]
        if len(sol) == 1:
            ob = bpy.data.objects.new(uniq(base), get_mesh(sol[0]["mesh"]))
        else:
            ob = bpy.data.objects.new(uniq(base), None)
            ob.empty_display_size = 0.01
            for s in sol:
                ch = bpy.data.objects.new(uniq(f"{base}~{s['solid'] if s['solid'] >= 0 else 99}"), get_mesh(s["mesh"]))
                col.objects.link(ch)
                ch.parent = ob
                ch["step_solid"] = s["solid"]
    ob["step_inst"] = nd["inst"]
    ob["step_prod"] = nd["prod"]
    col.objects.link(ob)
    if parent is not None:
        ob.parent = parent
        ob.matrix_parent_inverse = Matrix.Identity(4)
        ob.matrix_basis = M
    else:
        ob.matrix_basis = (Matrix(FIX) if FIX else Matrix.Identity(4)) @ M
    objs.append(ob)
    if i % 500 == 0:
        print(f"  {i}/{len(tree['nodes'])} nodes, {len(meshes)} meshes", flush=True)

bpy.context.view_layer.update()
mo = [o for o in col.objects if o.type == 'MESH']
tris = sum(sum(len(p.vertices) - 2 for p in o.data.polygons) for o in mo)
print(f"built {len(col.objects)} objects ({len(mo)} meshes, {len(meshes)} unique), {tris:,} scene tris")
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
