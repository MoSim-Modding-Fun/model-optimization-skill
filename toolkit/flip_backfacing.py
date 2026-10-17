# Flip faces of named objects that rays from outside hit on their BACK side first (finished-file patch). E.g. the
# library "Limelight 3" asset (GN booleans) has its two LED bore bottoms (16-gons, 9.6 mm2 each) wound inward, so with
# backface culling (Unity) the bores are see-through (review round 3: red backdrop shows through).
# Rays: 26 directions x a grid over the object's bounding sphere; a face is flipped when it is hit back-first by at
# least 3 rays and never front-first.
#   blender -b in.blend --python flip_backfacing.py -- out.blend "Minimal[|...]"   (out "-" = report only; names split on
#   "|" because wheel names contain commas; "@lib" = every object whose mesh came from the asset library)
import bpy, bmesh, sys, itertools
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree
a = sys.argv[sys.argv.index("--") + 1:]
out, names = a[0], a[1].split("|")
if names == ["@lib"]:
    names = [o.name for o in bpy.data.objects if o.type == 'MESH' and "[library" in o.data.name
             and not any(c.name.startswith("REVIEW") for c in o.users_collection)]
dirs = [Vector(d).normalized() for d in itertools.product((-1, 0, 1), repeat=3) if any(d)]
for n in names:
    o = bpy.data.objects[n]
    me = o.data
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.faces.ensure_lookup_table()
    wb = bm.copy()
    wb.transform(o.matrix_world)
    wb.normal_update()
    wb.faces.ensure_lookup_table()
    bvh = BVHTree.FromBMesh(wb)
    pts = np.array([v.co for v in wb.verts])
    ctr = Vector(((pts.min(0) + pts.max(0)) / 2).tolist())
    R = float(np.linalg.norm(pts.max(0) - pts.min(0)) / 2)
    back, front = {}, set()
    g = np.linspace(-R, R, 61)
    for d in dirs:
        u = d.orthogonal().normalized()
        w = d.cross(u)
        for s in g:
            for t in g:
                org = ctr - d * (2 * R) + u * s + w * t
                loc, nrm, fi, dist = bvh.ray_cast(org, d)
                if fi is None:
                    continue
                if wb.faces[fi].normal.dot(d) > 0:
                    back[fi] = back.get(fi, 0) + 1
                else:
                    front.add(fi)
    flip = [fi for fi, c in back.items() if c >= 3 and fi not in front]
    if out != "-":
        for fi in flip:
            bm.faces[fi].normal_flip()
    bm.to_mesh(me)
    me.update()
    print("FLIPBACK", n, me.name, "back-hit faces", len(back), "flipped", len(flip),
          [(fi, back[fi], round(wb.faces[fi].calc_area() * 1e6, 2)) for fi in flip], flush=True)
    bm.free()
    wb.free()
if out != "-":
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=out, compress=True)
