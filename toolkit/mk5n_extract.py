# One SDS MK5n module from a robot CAD into its own work file, in the MK4I asset's frame: L-plate outer corner at
# the origin, z = 0 at the bottom of the housing's base plate (= top of the frame rails; the L plate sits under them),
# module reaching into -x / +y. Every mesh is welded at 1 um (unwelded glTF) and the vendor's FUSED bodies ("SDS MK5
# Turret Assembly", "SDS MK5n Top Assembly (A)") are split into loose islands and then by colour (connected faces of one
# material), so each piece can be remodelled (lathe / plate / decimate) like a named part. Pieces are "<part>~NNN".
#   blender -b robot.blend --python mk5n_extract.py -- "SDS MK5n <1>" A out.blend
# Writes out_frame.json: T (robot world -> asset frame) for this module.
import bpy, bmesh, sys, os, math, json, collections
import numpy as np
from mathutils import Matrix
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build
root_name, tag, out = sys.argv[sys.argv.index("--") + 1:][:3]
FUSED = ("SDS MK5 Turret Assembly", "SDS MK5n Top Assembly", "MK5 Lifted Turret Assembly")  # last: the lifted revision


def module_frame(meshes):
    """Robot world -> asset frame from the L plate (outer corner, quarter turn into -x/+y) and the housing's base plate."""
    lp = max((o for o in meshes if o.name.startswith("SDS MK5 L Plate")), key=lambda o: o.dimensions.length)
    co = np.array([lp.matrix_world @ v.co for v in lp.data.vertices])
    lo, hi = co.min(0), co.max(0)
    corners = [(x, y) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])]
    near = [np.min(np.hypot(co[:, 0] - x, co[:, 1] - y)) for x, y in corners]
    empty = int(np.argmax(near))
    outer, inner = corners[3 - empty], corners[empty]   # the empty (inner) corner of the L is diagonal to the outer one
    d = np.array(inner) - np.array(outer)
    k = round((math.atan2(1, -1) - math.atan2(d[1], d[0])) / (math.pi / 2))
    # base-plate bottom = frame-rail top: the L plate sits under a 2 x 1 rail standing on edge, so L-plate top + 50.8 mm;
    # the housing must have a large downward face there (e.g. 95.3 mm, 5,324 mm2; a higher one at 106.4 mm is bigger)
    top = next(o for o in meshes if o.name.startswith("SDS MK5n Top Assembly"))
    M = np.array(top.matrix_world); R = M[:3, :3]
    acc = collections.Counter()
    for p in top.data.polygons:
        n = R @ np.array(p.normal)
        if n[2] < -0.99:
            acc[round(float((M @ np.array([*p.center, 1.0]))[2]), 4)] += p.area
    z0 = float(hi[2]) + 0.0508
    near_face = sum(a for z, a in acc.items() if abs(z - z0) < 2e-4)
    if near_face < 0.3 * max(acc.values()):
        print("WARNING base plate: no large housing face at L-plate top + 50.8 mm", round(z0 * 1000, 2), flush=True)
    T = Matrix.Rotation(k * math.pi / 2, 4, 'Z') @ Matrix.Translation((-outer[0], -outer[1], -z0))
    return T, outer, k * 90, z0


def split_islands(me):
    """Face groups of the welded mesh's connected islands (shared edges), largest first."""
    bm = bmesh.new(); bm.from_mesh(me); bm.faces.ensure_lookup_table()
    seen = np.zeros(len(bm.faces), bool); groups = []
    for f0 in bm.faces:
        if seen[f0.index]:
            continue
        stack = [f0]; seen[f0.index] = True; g = []
        while stack:
            f = stack.pop(); g.append(f.index)
            for e in f.edges:
                for f2 in e.link_faces:
                    if not seen[f2.index]:
                        seen[f2.index] = True; stack.append(f2)
        groups.append(np.array(sorted(g)))
    bm.free()
    return sorted(groups, key=lambda g: -len(g))


def main():
    sc = bpy.context.scene
    E = bpy.data.objects[root_name]
    meshes = [o for o in E.children_recursive if o.type == 'MESH' and len(o.data.polygons)]
    T, outer, rot, z0 = module_frame(meshes)
    print("EXTRACT", root_name, "outer corner mm", np.round(np.array(outer) * 1000, 2).tolist(), "rot deg", rot, "z0 mm", round(z0 * 1000, 2))
    W = {o.name: T @ o.matrix_world for o in meshes}
    keep = {o.name for o in meshes}
    for o in list(bpy.data.objects):
        if o.name not in keep:
            bpy.data.objects.remove(o)
    root = bpy.data.objects.new(f"MK5N {tag}", None); sc.collection.objects.link(root)
    import explode  # MeshArrays / piece_mesh: cut sub-meshes out of a big mesh, attributes and materials kept
    done = {}
    for o in meshes:
        o.data = o.data.copy()  # own copy (instanced meshes stay shared through `done` below)
    n_split = collections.Counter()
    for o in list(meshes):
        mat_w = W[o.name]
        build.weld(o.data, 1e-6)
        if not o.name.startswith(FUSED):
            o.parent = None; o.matrix_world = mat_w; o.parent = root; o.matrix_parent_inverse.identity(); o.matrix_world = mat_w
            continue
        groups = split_islands(o.data)
        A = explode.MeshArrays(o.data)
        base = o.name
        for i, g in enumerate(groups):
            pm = explode.piece_mesh(A, g, f"{o.data.name}~{i:03d}")
            po = bpy.data.objects.new(f"{base}~{i:03d}", pm); sc.collection.objects.link(po)
            po.parent = root; po.matrix_world = mat_w
            po["piece_of"] = base
            n_split[base] += 1
        bpy.data.objects.remove(o)
    for c in list(bpy.data.collections):
        bpy.data.collections.remove(c)
    for o in list(sc.collection.objects):
        pass
    bpy.data.orphans_purge(do_recursive=True)
    allco = np.array([o.matrix_world @ v.co for o in sc.objects if o.type == 'MESH' for v in o.data.vertices])
    print("EXTRACT", tag, "objects", sum(1 for o in sc.objects if o.type == 'MESH'), "split", dict(n_split),
          "bbox mm", np.round(allco.min(0) * 1000).tolist(), np.round(allco.max(0) * 1000).tolist(), flush=True)
    json.dump({"T": [list(r) for r in T], "root": root_name, "rot_deg": rot, "z0": float(z0), "outer": [float(x) for x in outer]},
              open(out.replace(".blend", "_frame.json"), "w"))
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=out, compress=True)


if __name__ == "__main__":
    main()
