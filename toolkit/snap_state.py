# Read-only: dump per-object state (mesh, tris after modifiers, modifiers, transform, collections, materials) to JSON
#   blender -b file.blend --python snap_state.py -- out.json
import bpy, sys, json
OUT = sys.argv[sys.argv.index("--") + 1]
dg = bpy.context.evaluated_depsgraph_get()
res = {}
for o in bpy.data.objects:
    d = {"type": o.type, "parent": o.parent.name if o.parent else None,
         "colls": sorted(c.name for c in o.users_collection),
         "mw": [round(x, 6) for r in o.matrix_world for x in r],
         "hide": [o.hide_viewport, o.hide_render, o.hide_get() if o.name in bpy.context.view_layer.objects else None]}
    if o.type == 'MESH':
        me = o.data
        d["mesh"] = me.name
        d["base_tris"] = sum(len(p.vertices) - 2 for p in me.polygons)
        d["nverts"] = len(me.vertices)
        d["mods"] = [[m.type, m.name] + ([m.decimate_type, round(m.ratio, 4), round(getattr(m, "angle_limit", 0), 4)] if m.type == 'DECIMATE' else [])
                     + ([m.node_group.name if m.node_group else None] if m.type == 'NODES' else []) + [m.show_viewport, m.show_render]
                     for m in o.modifiers]
        d["mats"] = [s.material.name if s.material else None for s in o.material_slots]
        ev = o.evaluated_get(dg).data
        d["tris"] = sum(len(p.vertices) - 2 for p in ev.polygons)
    res[o.name] = d
json.dump({"objects": res, "materials": sorted(m.name for m in bpy.data.materials),
           "colls": {c.name: len(c.objects) for c in bpy.data.collections}}, open(OUT, "w"), indent=0)
print("SNAP", len(res), flush=True)
