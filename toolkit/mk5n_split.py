# Library MK5n A/B into a robot's EXISTING SDS MK5n objects (2026-10-15): for a game model whose Unity prefab has
# the FBX unpacked, mk5n_place.py's new "<module> [MK5n A]" object would never reach the game. Here each library part
# goes into the CAD object it stands for, as that object's mesh in its own local frame:
#   cfg.MK5N_SPLIT = {"Housing": <CAD part>, "Turret": <CAD part>, "Wheel": <CAD part>}  (library part suffix -> CAD part)
# Library screws / spacer are left out (the robot keeps its own CAD screws and spacer, simplified by the pipeline), as are
# motors (library Krakens, own objects). The turret and wheel sit at the library's steer angle (owner: the wheel doesn't
# move in game, its angle can be anything). A or B per module by fit: the type whose housing lies closer to the module's
# current housing piece (median vertex distance). Objects sharing a CAD mesh share the new mesh (checked: the per-user
# results must agree within 0.05 mm).
#   blender -b simplified.blend --python mk5n_split.py -- <frames.json from mk5n_place.py frames> out.blend report.json
import bpy, sys, os, json
import numpy as np
from mathutils import Matrix
from mathutils.bvhtree import BVHTree
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import align, shading, robots
cfg = robots.load()
a = sys.argv[sys.argv.index("--") + 1:]
FRAMES, OUT, REPORT = json.load(open(a[0])), a[1], a[2]
LIBFILE = os.environ.get("MK5N_LIBFILE", f"{align.LIB}/MK5n Swerve Module/MK5n Swerve Module.blend")
SPLIT = cfg.MK5N_SPLIT
sc = bpy.context.scene

with bpy.data.libraries.load(LIBFILE, link=False) as (src, dst):
    dst.collections = [c for c in src.collections if c in ("MK5n A", "MK5n B")]
for c in dst.collections:
    sc.collection.children.link(c)
bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
lib = {}  # (type, kind) -> (mesh in the asset frame)
for c in dst.collections:
    tag = c.name.split()[-1]
    for o in c.all_objects:
        if o.type != 'MESH':
            continue
        kind = next((k for k in SPLIT if o.name.split(" - ")[-1].split(".")[0] == k), None)
        if kind is None:
            continue
        for m in o.modifiers:  # shading stack off: the receiving object gets its own
            if m.type == 'WEIGHTED_NORMAL' or (m.type == 'NODES' and m.node_group and m.node_group.name.startswith(shading.SBA)):
                m.show_viewport = m.show_render = False
        dg.update()
        me = bpy.data.meshes.new_from_object(o.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
        me.transform(o.matrix_world)  # asset frame (asset root at identity)
        lib[(tag, kind)] = me
print("MK5N SPLIT library parts", sorted(f"{t} {k} {len(m.polygons)}" for (t, k), m in lib.items()), flush=True)


def module_obj(E, part):
    # the CAD object itself, not an exploded piece ("~NNN") or an implode REVIEW child ("<object> [REVIEW - ...]")
    return next((o for o in E.children_recursive if o.type == 'MESH' and "~" not in o.name and "[" not in o.name
                 and cfg.part(o.name) == part), None)


def verts_world(me, M):
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    R = np.array(M)
    return co @ R[:3, :3].T + R[:3, 3]


rep = {"modules": {}, "meshes": {}}
REPL = bpy.data.collections.get("REVIEW - Replaced by library") or bpy.data.collections.new("REVIEW - Replaced by library")
if REPL.name not in sc.collection.children:
    sc.collection.children.link(REPL)
REPL.color_tag = 'COLOR_04'
new_for = {}  # CAD mesh name -> (new mesh, first user, local matrix)
for name, f in sorted(FRAMES.items()):
    E = bpy.data.objects[name]
    Tinv = Matrix(f["T"]).inverted()  # asset frame -> world
    house = module_obj(E, SPLIT["Housing"])
    eh = house.evaluated_get(bpy.context.evaluated_depsgraph_get())
    ev = eh.to_mesh()
    hv = verts_world(ev, house.matrix_world)
    bvh = BVHTree.FromPolygons([tuple(v) for v in hv], [tuple(p.vertices) for p in ev.polygons])
    eh.to_mesh_clear()
    score = {}
    for tag in ("A", "B"):
        if (tag, "Housing") not in lib:
            continue
        pts = verts_world(lib[(tag, "Housing")], Tinv)[::2]
        d = np.array([bvh.find_nearest(p)[3] for p in pts]) * 1000
        score[tag] = (float(np.median(d)), float(np.percentile(d, 90)))
    tag = min(score, key=lambda t: score[t][0])
    rep["modules"][name] = {"type": tag, "housing_fit_mm": score}
    print("MK5N SPLIT", name, "->", tag, "housing median/p90 mm", {k: [round(x, 2) for x in v] for k, v in score.items()}, flush=True)
    for kind, part in SPLIT.items():
        o = module_obj(E, part)
        if o is None:
            print("MK5N SPLIT missing", part, "under", name, flush=True)
            continue
        Mloc = o.matrix_world.inverted() @ Tinv
        cad = o.data.name
        if cad in new_for:  # shared CAD mesh: the result must agree with the first user's
            me0, first, M0 = new_for[cad]
            dv = np.abs(verts_world(lib[(tag, kind)], Mloc) - verts_world(lib[(tag, kind)], M0)).max() * 1000
            rep["meshes"].setdefault(cad, {})["dev_mm_" + o.name] = round(float(dv), 3)
            if dv > 0.05:
                print("MK5N SPLIT WARNING", o.name, "differs from", first, "by", round(dv, 3), "mm (shares", cad, ")", flush=True)
            o.data = me0
        else:
            me = lib[(tag, kind)].copy()
            me.transform(Mloc)
            me.name = cad + " [MK5n " + tag + " " + kind + "]"
            new_for[cad] = (me, o.name, Mloc)
            rep["meshes"][cad] = {"kind": kind, "type": tag, "tris": sum(len(p.vertices) - 2 for p in me.polygons),
                                  "old_tris": sum(len(p.vertices) - 2 for p in o.data.polygons)}
            o.data = me
        # the per-piece mesh was a live Collapse / dissolve stack on some pieces: the library mesh is final
        for m in list(o.modifiers):
            o.modifiers.remove(m)
        shading.add_shading(o)
        o["mk5n_library"] = f"MK5n {tag} {kind}"
        # implode.py's "<object> [REVIEW - ...]" children (the old per-piece hidden pieces) are what the library part
        # stands for now
        for ch in o.children:
            if ch.name.startswith(o.name + " [REVIEW"):
                for c in list(ch.users_collection):
                    c.objects.unlink(ch)
                REPL.objects.link(ch)
                rep.setdefault("replaced", []).append(ch.name)
for c in dst.collections:
    for o in list(c.all_objects):
        bpy.data.objects.remove(o)
    bpy.data.collections.remove(c)
for me in lib.values():
    bpy.data.meshes.remove(me)
json.dump(rep, open(REPORT, "w"), indent=1)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
print("MK5N SPLIT saved", OUT, flush=True)
