# Z-fight fixes on a finished piece-level file (zfight.py "ZF new" pairs), deterministic, no rebuild (Collapse is not
# repeatable, so a rerun would bring new pairs):
#   faces: A's faces lying in B's surface (within 0.05 mm, facing the same way) -> every vertex of A on that plane moves
#          `mm` into A (lathe / hex / plate end caps pulled in; side walls get that much shorter). Planes come from face
#          centre and corner samples plus area samples (overlaps along an edge strip). A must have no geometry modifiers.
#   move:  A (a library object) translated `mm` along the shared face normal (+ = out of A's face, - = into it). Check
#          the far end too: a motor moved off a module face can land its shaft end on a pinion face.
# Move the smaller part (insert, bearing, bolt, nut stub); keep each move under a quarter of the part's thickness.
# Prints the planes, the vertex count moved and A's hits in B before -> after (1500 area samples); rerun zfight.py after.
#   blender -b <run>/<piece file>.blend --python zf_nudge.py -- <pairs.json> <out.blend>
#   pairs.json: [{"a": obj, "b": obj, "mm": 0.15, "mode": "faces"|"move"}, ...]
import bpy, sys, json
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree
a = sys.argv[sys.argv.index("--") + 1:]
PAIRS, OUT = json.load(open(a[0])), a[1]
GEOM_MODS = {'DECIMATE', 'BOOLEAN', 'SOLIDIFY', 'REMESH', 'BEVEL', 'ARRAY', 'MIRROR', 'SUBSURF'}
TOL, NDOT = 5e-5, 0.9995


def bvh_of(o):
    dg = bpy.context.evaluated_depsgraph_get()
    ev = o.evaluated_get(dg); m = ev.to_mesh(); m.calc_loop_triangles()
    M = o.matrix_world
    V = [M @ v.co for v in m.vertices]; T = [tuple(t.vertices) for t in m.loop_triangles]
    N = [((V[t[1]] - V[t[0]]).cross(V[t[2]] - V[t[0]])).normalized() for t in T]
    ev.to_mesh_clear()
    return BVHTree.FromPolygons(V, T), N


def planes(A, B):
    """A's face planes (world normal, offset) that lie in B's surface facing the same way, with the hit count (faces whose
    centre or a corner sample lies in B, plus area samples)."""
    bv, NB = bvh_of(B)
    M = A.matrix_world; R = M.to_3x3()
    me = A.data
    found = {}
    for p in me.polygons:
        n = (R @ p.normal).normalized()
        c = M @ p.center
        pts = [c] + [c + ((M @ me.vertices[i].co) - c) * 0.7 for i in p.vertices]
        for q in pts:
            loc, _, fi, _ = bv.find_nearest(q, TOL)
            if loc is not None and n.dot(NB[fi]) > NDOT:
                k = (tuple(round(x, 3) for x in n), round(n.dot(c) * 1e5))  # 0.01 mm bins
                f = found.setdefault(k, [n, n.dot(c), 0]); f[2] += 1
                break
    hits(A, B, 3000, found)
    merged = []  # samples of one plane land in neighbouring 0.01 mm bins
    for n, d, c in sorted(found.values(), key=lambda f: -f[2]):
        for m in merged:
            if n.dot(m[0]) > NDOT and abs(d - m[1]) < 2e-5:
                m[2] += c; break
        else:
            merged.append([n, d, c])
    return merged


def hits(A, B, n=1500, found=None):
    """A's area-weighted samples lying in B's surface (same facing): a before/after measure; with `found`, the planes of
    the hit samples are added to it (overlaps along an edge strip that no face centre or corner sample reaches)."""
    import random
    bv, NB = bvh_of(B)
    dg = bpy.context.evaluated_depsgraph_get()
    ev = A.evaluated_get(dg); m = ev.to_mesh(); m.calc_loop_triangles()
    M = A.matrix_world
    V = [M @ v.co for v in m.vertices]; T = [tuple(t.vertices) for t in m.loop_triangles]
    ev.to_mesh_clear()
    ar = [((V[t[1]] - V[t[0]]).cross(V[t[2]] - V[t[0]])).length for t in T]
    random.seed(7); h = 0
    for i in random.choices(range(len(T)), weights=ar, k=n):
        t = T[i]; r1, r2 = random.random(), random.random()
        if r1 + r2 > 1: r1, r2 = 1 - r1, 1 - r2
        q = V[t[0]] + (V[t[1]] - V[t[0]]) * r1 + (V[t[2]] - V[t[0]]) * r2
        nn = (V[t[1]] - V[t[0]]).cross(V[t[2]] - V[t[0]]).normalized()
        loc, _, fi, _ = bv.find_nearest(q, TOL)
        if loc is not None and nn.dot(NB[fi]) > NDOT:
            h += 1
            if found is not None:
                k = (tuple(round(x, 3) for x in nn), round(nn.dot(q) * 1e5))
                f = found.setdefault(k, [nn, nn.dot(q), 0]); f[2] += 1
    return h


for e in PAIRS:
    A, B = bpy.data.objects.get(e["a"]), bpy.data.objects.get(e["b"])
    if A is None or B is None:
        print("NUDGE missing", e["a"] if A is None else e["b"], flush=True); continue
    bad = [m.type for m in A.modifiers if m.type in GEOM_MODS]
    if bad and e["mode"] == "faces":
        print("NUDGE skip (geometry modifiers)", A.name, bad, flush=True); continue
    h0 = hits(A, B)
    pl = planes(A, B)
    if not pl:
        print("NUDGE no plane", A.name, "|", B.name, "hits", h0, flush=True); continue
    mm = e["mm"] * 1e-3
    if e["mode"] == "move":
        n = Vector(np.mean([np.array(x[0]) for x in pl], axis=0)).normalized()
        A.matrix_world = __import__("mathutils").Matrix.Translation(n * mm) @ A.matrix_world
        moved = "object"
    else:
        if A.data.users > 1 or A.data.library is not None:
            A.data = A.data.copy()
        me = A.data
        M = A.matrix_world; Mi = M.inverted(); Ri = Mi.to_3x3()
        co = np.array([M @ v.co for v in me.vertices])
        moved = 0
        for n, d, _ in pl:
            nn = np.array(n)
            on = np.abs(co @ nn - d) < 2e-5
            dl = Ri @ (n * -mm)
            for i in np.nonzero(on)[0]:
                me.vertices[int(i)].co += dl
            moved += int(on.sum())
        me.update()
    bpy.context.view_layer.update()
    h1 = hits(A, B)
    print("NUDGE", e["mode"], A.name[:60], "|", B.name[:50], "planes", [(tuple(round(x, 2) for x in n), round(d * 1e3, 2), c) for n, d, c in pl],
          "moved", moved, "hits", h0, "->", h1, flush=True)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=False)
print("NUDGE saved", OUT, flush=True)
