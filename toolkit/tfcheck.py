# Names, parents and world matrices of the deliverable vs the source (prep.py writes tf_src.json; or dump it here).
#   blender -b "<source>.blend" --python tfcheck.py -- dump tf_src.json
#   blender -b "<name> - simplified.blend" --python tfcheck.py -- check tf_src.json [orig_names.json]
# check prints: objects compared, the largest world-matrix deviation (must be 0.0 apart from float noise), objects that
# moved more than 1e-6, source objects missing from the deliverable (expected only where the owner deleted them), and
# the root objects with their rotation (a work copy with a rotated root must be restored exactly).
import bpy, sys, json
import numpy as np
a = sys.argv[sys.argv.index("--") + 1:]
if a[0] == "dump":
    json.dump({o.name: [list(r) for r in o.matrix_world] for o in bpy.data.objects}, open(a[1], "w"))
    print("TFCHECK dumped", len(bpy.data.objects))
else:
    src = json.load(open(a[1]))
    worst, n, bad, missing = 0.0, 0, [], []
    for name, M in src.items():
        o = bpy.data.objects.get(name)
        if o is None:
            missing.append(name)
            continue
        d = float(np.abs(np.array(o.matrix_world) - np.array(M)).max())
        n += 1
        worst = max(worst, d)
        if d > 1e-6:
            bad.append((name, round(d, 6)))
    print("TFCHECK compared", n, "of", len(src), "objects; max world-matrix deviation", worst, "; moved > 1e-6:",
          len(bad), bad[:10])
    print("TFCHECK missing", len(missing), missing[:20])
    if len(a) > 2:
        orig = set(json.load(open(a[2])))
        added = sorted(set(o.name for o in bpy.data.objects) - orig)
        print("TFCHECK added", len(added), added[:20])
    print("TFCHECK roots", [(o.name, o.rotation_mode, [round(x, 6) for x in o.rotation_quaternion])
                            for o in bpy.data.objects if o.parent is None and o.type == 'EMPTY'][:5])
