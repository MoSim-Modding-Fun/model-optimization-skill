# Phase 1 planning (read-only): unique geometries (hash of local verts/topology/materials) and their tris.
#   blender -b robot.blend --python cleanup/plan.py -- units.json
import bpy, sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import clean_core as cc
OUT = sys.argv[sys.argv.index("--") + 1]
groups = {}
for o in bpy.context.scene.objects:
    if o.type == 'MESH' and o.data.name not in {m for g in groups.values() for m in g["meshes"]}:
        h = cc.geo_hash(o.data)
        g = groups.setdefault(h, {"meshes": [], "tris": cc.tris(o.data)})
        g["meshes"].append(o.data.name)
json.dump([{"hash": h, "lead": sorted(g["meshes"])[0], "copies": sorted(g["meshes"])[1:], "tris": g["tris"]} for h, g in groups.items()],
          open(OUT, "w"))
print("PLAN", len(groups), "unique geometries", sum(g["tris"] for g in groups.values()), "unique tris", flush=True)
