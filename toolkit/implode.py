# Undo explode.py after the pipeline: each exploded object gets its processed visible pieces joined back into its own
# mesh (name, parent, transform unchanged). Pieces the pipeline moved into a REVIEW collection are joined into one
# "<object> [<review>]" child object in that collection, so nothing is deleted. Live Decimate is applied; the
# Smooth by Angle -> WeightedNormal stack goes onto the joined object instead.
#   ROBOT=<robot> blender -b simplified_exploded.blend --python implode.py -- out.blend
import bpy, sys, os
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import shading, libcombo
OUT = sys.argv[sys.argv.index("--") + 1]
sc = bpy.context.scene
bpy.context.preferences.filepaths.save_version = 0


def is_shading(m):
    return m.type == 'WEIGHTED_NORMAL' or (m.type == 'NODES' and m.node_group and m.node_group.name.startswith(shading.SBA))


def piece_mesh(po):
    """Evaluated piece without the shading stack (Decimate etc. applied)."""
    for m in po.modifiers:
        if is_shading(m):
            m.show_viewport = m.show_render = False
    dg = bpy.context.evaluated_depsgraph_get()
    return bpy.data.meshes.new_from_object(po.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)


cache = {}
report = []
for o in [o for o in sc.objects if "exploded_from" in o]:
    pieces = [p for p in o.children if p.get("piece_of") == o.name]
    groups = {}
    for p in pieces:
        rev = next((c.name for c in p.users_collection if c.name.startswith("REVIEW")), None)
        groups.setdefault(rev, []).append(p)
    for rev, ps in groups.items():
        # piece matrices are part of the key: instanced pieces (explode with shared meshes) of two objects can use the
        # same meshes at different places
        key = (rev, tuple(sorted((p.data.name, tuple(round(v, 7) for r_ in p.matrix_basis for v in r_)) for p in ps)),
               tuple(sorted(m.type for p in ps for m in p.modifiers)))
        if key not in cache:
            parts = [(piece_mesh(p), p.matrix_basis.copy()) for p in ps]  # piece transforms relative to o
            cache[key] = libcombo.join_meshes(f"{o['exploded_from']}" + (f" [{rev}]" if rev else ""), parts)
            for me, _ in parts:
                bpy.data.meshes.remove(me)
        shade = any(is_shading(m) for p in ps for m in p.modifiers)
        if rev is None:
            old = o.data
            o.data = cache[key]
            if old.users == 0:
                bpy.data.meshes.remove(old)
            target = o
        else:
            target = bpy.data.objects.new(f"{o.name} [{rev}]", cache[key])
            bpy.data.collections[rev].objects.link(target)
            target.parent = o
            target.matrix_parent_inverse.identity()
            target.matrix_basis.identity()
        shading.remove_shading(target)
        if shade:
            shading.add_shading(target)
        report.append((o.name, rev, len(ps), sum(len(q.vertices) - 2 for q in cache[key].polygons)))
    for p in pieces:
        me = p.data
        bpy.data.objects.remove(p)
        if me.users == 0:
            bpy.data.meshes.remove(me)
    del o["exploded_from"]
for r in report:
    print("IMPLODE", r, flush=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
print("IMPLODE saved", OUT, flush=True)
