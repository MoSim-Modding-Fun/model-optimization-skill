# Drop-in FBX for a game model whose Unity prefab has the FBX unpacked (2026-10-15; see game_restore.py).
# Blender's FBX export with live modifiers (Apply Modifiers, on by default) gives EVERY modified object its own
# geometry named "<mesh>.002", ".003", ... and drops the mesh sharing, so Unity would see new meshes and the prefab's mesh
# references would break. This bakes the modifier stack (Smooth by Angle -> WeightedNormal, live Decimates) into each
# shared mesh once, keeping its name, then exports everything (REVIEW collections included) with default settings.
#   export:   blender -b restored.blend --python fbx_dropin.py -- export out.fbx [baked.blend]
#   compare:  blender -b --factory-startup --python fbx_dropin.py -- compare game.fbx out.fbx report.json
# compare checks the scene the game sees: GlobalSettings (units, axes), every Model's name, type, parent and local
# transform (Lcl Translation / Rotation / Scaling, pre/post rotation), every Geometry's name and which Models use it,
# and the material count per Model (= the prefab renderer's material list length).
import bpy, sys, os, json, math

a = sys.argv[sys.argv.index("--") + 1:]
MODE = a[0]


def is_live(m):
    return m.show_viewport or m.show_render


if MODE == "export":
    OUT = a[1]
    users = {}
    for o in bpy.data.objects:
        if o.type == 'MESH':
            users.setdefault(o.data.name, []).append(o)
    dg = bpy.context.evaluated_depsgraph_get()
    n_baked, errs = 0, []
    for mname, os_ in users.items():
        stacks = [tuple((m.type, m.name) for m in o.modifiers if is_live(m)) for o in os_]
        if not any(stacks):
            for o in os_:
                o.modifiers.clear()  # disabled leftovers: nothing to bake
            continue
        if len(set(stacks)) > 1:
            errs.append(f"{mname}: users carry different modifier stacks {sorted(set(stacks))}")
        # bake from the user with the fullest stack: a CAD duplicate moved to REVIEW carries none, and as the
        # first user it dropped the shading from every user of that mesh
        src = max(os_, key=lambda o: len([m for m in o.modifiers if is_live(m)]))
        new = bpy.data.meshes.new_from_object(src.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
        old = bpy.data.meshes[mname]
        old.name = mname + " __live"
        new.name = mname
        if new.name != mname:
            errs.append(f"rename failed: {new.name}")
        for o in os_:
            o.data = new
            o.modifiers.clear()
        n_baked += 1
    for lc in bpy.context.view_layer.layer_collection.children:  # export everything, REVIEW included
        lc.exclude = False
    for o in bpy.data.objects:
        o.hide_set(False)
        o.hide_viewport = False
    bpy.context.view_layer.update()
    print("DROPIN baked", n_baked, "meshes; errors", errs)
    # FBX_OPTS (JSON, optional): exporter options that reproduce the game FBX's GlobalSettings. One game FBX has
    # UnitScaleFactor 100 with root scale 1: '{"apply_scale_options": "FBX_SCALE_UNITS"}' (the default writes 1 and
    # scales the root by 100)
    # FBX_ORDER (optional): path of the game FBX. Unity names a shared mesh after its FIRST connected Model, and
    # Blender's exporter writes objects sorted by name, which is not always the game FBX's order ("Hex nut M3x0.001"
    # sorts before "Hex nut M3x0.5"; 7 of 625 meshes changed their first user in a plain round trip). With it the
    # objects are written in the game FBX's Model order (new objects after them, by name).
    if os.environ.get("FBX_ORDER"):
        from io_scene_fbx import export_fbx_bin, parse_fbx as _pf
        _root, _ = _pf.parse(os.environ["FBX_ORDER"])
        _objs = next(e for e in _root.elems if e.id == b"Objects")
        _rank = {}
        for e in _objs.elems:
            if e.id == b"Model":
                _rank.setdefault(e.props[1].split(b"\x00\x01")[0].decode("utf-8", "replace"), len(_rank))
        _save_single = export_fbx_bin.save_single

        def _ordered(*args, **kw):
            kw["context_objects"] = sorted(kw["context_objects"], key=lambda o: (_rank.get(o.name, len(_rank)), o.name))
            return _save_single(*args, **kw)
        export_fbx_bin.save_single = _ordered
    bpy.ops.export_scene.fbx(filepath=OUT, **json.loads(os.environ.get("FBX_OPTS", "{}")))
    print("DROPIN fbx", OUT, os.path.getsize(OUT))
    if len(a) > 2:
        bpy.context.preferences.filepaths.save_version = 0
        bpy.ops.wm.save_as_mainfile(filepath=a[2], compress=True)
    raise SystemExit(0)

# ------------------------------------------------------------------ compare two FBX files
from io_scene_fbx import parse_fbx


def nm(b):
    return b.split(b"\x00\x01")[0].decode("utf-8", "replace")


def props70(e):
    out = {}
    p = next((x for x in e.elems if x.id == b"Properties70"), None)
    if p is None:
        return out
    for q in p.elems:
        key = q.props[0].decode()
        out[key] = q.props[4:]
    return out


def scene(path):
    root, ver = parse_fbx.parse(path)
    secs = {e.id: e for e in root.elems}
    gs = {}
    g = secs.get(b"GlobalSettings")
    if g is not None:
        gs = props70(g)
    models, geoms, mats = {}, {}, {}
    for e in secs[b"Objects"].elems:
        uid = e.props[0]
        if e.id == b"Model":
            p = props70(e)
            models[uid] = {"name": nm(e.props[1]), "type": e.props[2].decode(),
                           "T": list(p.get("Lcl Translation", (0, 0, 0))), "R": list(p.get("Lcl Rotation", (0, 0, 0))),
                           "S": list(p.get("Lcl Scaling", (1, 1, 1))), "pre": list(p.get("PreRotation", (0, 0, 0))),
                           "post": list(p.get("PostRotation", (0, 0, 0))), "rotorder": p.get("RotationOrder", (0,))[0] if p.get("RotationOrder") else 0}
        elif e.id == b"Geometry":
            geoms[uid] = nm(e.props[1])
        elif e.id == b"Material":
            mats[uid] = nm(e.props[1])
    parent, geo_of, mats_of, first_of = {}, {}, {}, {}
    for c in secs[b"Connections"].elems:
        if c.props[0] != b"OO":
            continue
        ch, pa = c.props[1], c.props[2]
        if ch in models and (pa in models or pa == 0):
            parent[ch] = models[pa]["name"] if pa in models else None
        elif ch in geoms and pa in models:
            geo_of[models[pa]["name"]] = geoms[ch]
            first_of.setdefault(geoms[ch], models[pa]["name"])  # Unity names the mesh after this Model
        elif ch in mats and pa in models:
            mats_of.setdefault(models[pa]["name"], []).append(mats[ch])
    by_name = {}
    for uid, m in models.items():
        m["parent"] = parent.get(uid)
        m["geometry"] = geo_of.get(m["name"])
        m["materials"] = mats_of.get(m["name"], [])
        by_name.setdefault(m["name"], []).append(m)
    keep = ("UpAxis", "UpAxisSign", "FrontAxis", "FrontAxisSign", "CoordAxis", "CoordAxisSign", "UnitScaleFactor",
            "OriginalUnitScaleFactor")
    return {"global": {k: list(gs[k]) for k in keep if k in gs}, "models": by_name, "n_geoms": len(geoms), "first": first_of}


def rot_dev(e1, e2):
    from mathutils import Euler
    m1 = Euler([math.radians(x) for x in e1], 'XYZ').to_matrix()
    m2 = Euler([math.radians(x) for x in e2], 'XYZ').to_matrix()
    return max(abs(m1[i][j] - m2[i][j]) for i in range(3) for j in range(3))


A, B, REP = a[1], a[2], a[3]
sa, sb = scene(A), scene(B)
rep = {"global_a": sa["global"], "global_b": sb["global"], "global_same": sa["global"] == sb["global"],
       "models_a": sum(len(v) for v in sa["models"].values()), "models_b": sum(len(v) for v in sb["models"].values()),
       "geoms_a": sa["n_geoms"], "geoms_b": sb["n_geoms"]}
missing = sorted(n for n in sa["models"] if n not in sb["models"])
added = sorted(n for n in sb["models"] if n not in sa["models"])
dup = sorted(n for n, v in sa["models"].items() if len(v) > 1)
diffs = {"type": [], "parent": [], "transform": [], "geometry": [], "material_count": [], "material_names": []}
share_a, share_b = {}, {}
for n, va in sa["models"].items():
    if n not in sb["models"]:
        continue
    ma, mb = va[0], sb["models"][n][0]
    if ma["type"] != mb["type"]:
        diffs["type"].append([n, ma["type"], mb["type"]])
    if ma["parent"] != mb["parent"]:
        diffs["parent"].append([n, ma["parent"], mb["parent"]])
    # Euler angles compared as rotation matrices (a re-export writes -180 / 180 / 360 wrap-arounds of the same rotation);
    # translation / scale in FBX units (cm here): 1e-3 = 10 um
    d = max(abs(x - y) for k in ("T", "S") for x, y in zip(ma[k], mb[k]))
    dr = max(rot_dev(ma[k], mb[k]) for k in ("R", "pre", "post"))
    if d > 1e-3 or dr > 1e-4 or ma["rotorder"] != mb["rotorder"]:
        diffs["transform"].append([n, round(d, 6), round(dr, 8)])
    if ma["geometry"] != mb["geometry"]:
        diffs["geometry"].append([n, ma["geometry"], mb["geometry"]])
    if len(ma["materials"]) != len(mb["materials"]):
        diffs["material_count"].append([n, len(ma["materials"]), len(mb["materials"])])
    elif ma["materials"] != mb["materials"]:
        diffs["material_names"].append([n, ma["materials"], mb["materials"]])
    if ma["geometry"]:
        share_a.setdefault(ma["geometry"], set()).add(n)
    if mb["geometry"]:
        share_b.setdefault(mb["geometry"], set()).add(n)
share_diff = sorted(g for g in share_a if share_a[g] != share_b.get(g))
# the first connected Model of every geometry both files have. Only an early warning: Unity names the mesh (and
# its fileID) after the first node of its own hierarchy walk; run the Unity probe (unity_probe.sh) to be sure
first_diff = sorted([g, sa["first"][g], sb["first"][g]] for g in sa["first"] if g in sb["first"] and sa["first"][g] != sb["first"][g])
rep.update({"missing_models": missing, "added_models": added, "duplicate_names_in_a": dup,
            "diff_counts": {k: len(v) for k, v in diffs.items()}, "diffs": diffs, "sharing_diffs": share_diff,
            "first_model_diffs": first_diff})
json.dump(rep, open(REP, "w"), indent=1)
print("COMPARE global_same", rep["global_same"], "models", rep["models_a"], "->", rep["models_b"], "geoms", rep["geoms_a"], "->",
      rep["geoms_b"], "missing", len(missing), "added", len(added), "diffs", rep["diff_counts"], "sharing_diffs", len(share_diff), "first_model_diffs", len(first_diff))
