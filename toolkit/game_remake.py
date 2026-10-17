# After the owner has reviewed a delivered drop-in model and deleted objects (REVIEW collections), make the file
# export-safe again for a Unity prefab that has the FBX unpacked (2026-10-15; owner ruling: no
# prefab edits, and no Missing meshes after the re-export).
#   blender -b "<robot> - simplified.blend" --python game_remake.py -- orig_map.json orig.blend out.blend report.json
# orig_map.json is game_restore.py's dump of the ORIGINAL import (every object, parent, local matrix, mesh, slots).
# For every original object that is gone from the file:
#   - empties come back as empties (same name, parent, local matrix);
#   - a mesh object whose mesh still exists (shared with a kept object) comes back as a node on that mesh: the prefab's
#     GameObject points at the shared mesh, so it can't be hidden in game anyway (listed as "still visible in game"), and
#     Unity names a shared mesh after its FIRST FBX node, so a missing first user would rename it for the others;
#   - a mesh object whose mesh is gone gets a placeholder mesh under the ORIGINAL mesh name: one 0.02 mm triangle per
#     material slot at the object's origin, same slot list (the renderer's materials still line up, nothing to see).
# Everything comes back into the collection "DELETED - placeholders for the game". Then run game_restore.py (restore
# mode) and fbx_dropin.py (export + compare) on the result as usual.
import bpy, sys, json, math
from mathutils import Matrix, Vector

a = sys.argv[sys.argv.index("--") + 1:]
ORIG = json.load(open(a[0]))
ORIG_BLEND, OUT, REPORT = a[1], a[2], a[3]
sc = bpy.context.scene
objs = bpy.data.objects
coll = bpy.data.collections.get("DELETED - placeholders for the game") or bpy.data.collections.new("DELETED - placeholders for the game")
if coll.name not in sc.collection.children:
    sc.collection.children.link(coll)
coll.color_tag = 'COLOR_08'
rep = {"recreated_empties": [], "recreated_on_shared_mesh": [], "placeholders": [], "materials_restored": []}

gone = [n for n in ORIG["objects"] if n not in objs]
need_mats = sorted({m for n in gone if ORIG["objects"][n]["mesh"] for m in ORIG["meshes"][ORIG["objects"][n]["mesh"]]["mats"]
                    if m and m not in bpy.data.materials})
if need_mats:  # materials only the deleted objects used (the FBX's embedded materials are referenced by name)
    with bpy.data.libraries.load(ORIG_BLEND, link=False) as (src, dst):
        dst.materials = [m for m in need_mats if m in src.materials]
    rep["materials_restored"] = need_mats


def depth(n):
    d, p = 0, ORIG["objects"][n]["parent"]
    while p:
        d += 1
        p = ORIG["objects"][p]["parent"]
    return d


placeholder = {}
for n in sorted(gone, key=depth):
    info = ORIG["objects"][n]
    data = None
    if info["type"] == 'MESH':
        M = info["mesh"]
        me = bpy.data.meshes.get(M)
        if me is not None and me.users:
            data = me
            rep["recreated_on_shared_mesh"].append([n, M])
        else:
            if M not in placeholder:
                if me is not None:  # an orphan datablock still holds the name
                    me.name = M + " __old"
                mats = ORIG["meshes"][M]["mats"]
                verts, faces = [], []
                r = 1e-5
                for k in range(max(1, len(mats))):
                    base = len(verts)
                    for j in range(3):
                        ang = j * 2 * math.pi / 3
                        verts.append((r * math.cos(ang), r * math.sin(ang), k * 3 * r))
                    faces.append((base, base + 1, base + 2))
                pm = bpy.data.meshes.new(M)
                pm.from_pydata(verts, [], faces)
                for m in mats:
                    pm.materials.append(bpy.data.materials.get(m) if m else None)
                pm.polygons.foreach_set("material_index", list(range(len(faces))) if mats else [0])
                pm.update()
                placeholder[M] = pm
                rep["placeholders"].append([M, len(mats)])
            data = placeholder[M]
    elif info["type"] != 'EMPTY':
        print("REMAKE skip", n, info["type"])
        continue
    o = bpy.data.objects.new(n, data)
    if o.name != n:
        print("REMAKE name clash", n, "->", o.name)
    coll.objects.link(o)
    if info["parent"]:
        o.parent = objs[info["parent"]]
        o.matrix_parent_inverse = Matrix.Identity(4)
    o.matrix_basis = Matrix(info["basis"])
    if info["type"] == 'EMPTY':
        rep["recreated_empties"].append(n)
rep["summary"] = {"gone": len(gone), "empties": len(rep["recreated_empties"]),
                  "on_shared_mesh_still_visible_in_game": len(rep["recreated_on_shared_mesh"]),
                  "placeholder_meshes": len(rep["placeholders"])}
json.dump(rep, open(REPORT, "w"), indent=1)
print("REMAKE", json.dumps(rep["summary"]))
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
