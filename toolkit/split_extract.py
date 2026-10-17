# One fused CAD mesh (a vendor wheel exported as a single body, e.g. a 6" ION omni) into a work file for the
# per-piece pipeline, in the mesh's OWN frame (identity object transform): welded at 1 um, split by material, then by
# connected islands; pieces are named "<label> <k>" by material slot k ("Omni 0.003"), largest first.
#   blender -b robot.blend --python split_extract.py -- "<object name>" <label> <out.blend>
import bpy, bmesh, sys, os
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build
a = sys.argv[sys.argv.index("--") + 1:]
src, label, out = a[:3]
o = bpy.data.objects[src]
me = o.data.copy()
build.weld(me, 1e-6)
mats = list(me.materials)
pieces = []
for k in range(max(1, len(mats))):
    bm = bmesh.new(); bm.from_mesh(me)
    bmesh.ops.delete(bm, geom=[f for f in bm.faces if f.material_index != k], context='FACES')
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context='VERTS')
    seen, comps = set(), []
    for f in bm.faces:
        if f in seen:
            continue
        stack, comp = [f], []
        seen.add(f)
        while stack:
            g = stack.pop(); comp.append(g)
            for e in g.edges:
                for h in e.link_faces:
                    if h not in seen:
                        seen.add(h); stack.append(h)
        comps.append(comp)
    for comp in sorted(comps, key=len, reverse=True):
        keep = set(comp)
        b2 = bm.copy()
        idx = {f.index for f in comp}
        b2.faces.ensure_lookup_table()
        bmesh.ops.delete(b2, geom=[f for f in b2.faces if f.index not in idx], context='FACES')
        bmesh.ops.delete(b2, geom=[v for v in b2.verts if not v.link_faces], context='VERTS')
        pm = bpy.data.meshes.new(f"{label} {k}")
        b2.to_mesh(pm); b2.free()
        for m in mats:
            pm.materials.append(m)
        pieces.append(pm)
    bm.free()
for ob in list(bpy.data.objects):
    bpy.data.objects.remove(ob)
for c in list(bpy.data.collections):
    bpy.data.collections.remove(c)
root = bpy.data.objects.new(label.upper(), None)
bpy.context.scene.collection.objects.link(root)
for pm in pieces:
    ob = bpy.data.objects.new(pm.name, pm)
    bpy.context.scene.collection.objects.link(ob)
    ob.parent = root
print("SPLIT", src, "->", len(pieces), "pieces:", sorted({p.name.rsplit('.', 1)[0] for p in pieces}),
      [len(p.polygons) for p in pieces][:12], flush=True)
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=out, compress=True)
