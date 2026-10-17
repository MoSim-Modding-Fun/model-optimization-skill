# Hex stock that came out round. Owner rule: hex shafts and rounded-hex stock are exact 6-sided prisms; four review
# rounds once missed six 1/2" hex shafts lathed into 16-gons (a 16-gon reads as hex at sheet size). Per elongated CAD mesh
# (length > 2x diameter, 4-60 mm across): outer radius per 5 deg in thin slices at 1/4, 1/2, 3/4 length (area-weighted
# surface samples: long side faces have no vertices mid-length), amplitude of its 6th harmonic. Hex stock shows 0.3-0.45 mm
# (1/2" and 3/8" hex) with the other harmonics < half of it; the same measure on the simplified object should match.
#   blender -b "robot - simplified.blend" --python hexcheck.py -- <CAD.blend> <out.json>
# Prints HEXCHECK lines; "ROUND" = CAD hex, output not (force "hex" or "stacked lathe" for that mesh).
import bpy, sys, json, math
import numpy as np
a = sys.argv[sys.argv.index("--") + 1:]
CAD, OUT = a[0], a[1]


def harmonics(co, tri, rng):
    """(6th-harmonic amplitude, largest other harmonic, mean radius) in m, or None if not elongated stock."""
    c = co.mean(0)
    w, ev = np.linalg.eigh(np.cov((co - c).T))
    ax = ev[:, 2]
    A, B, C = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
    ar = 0.5 * np.linalg.norm(np.cross(B - A, C - A), axis=1)
    if ar.sum() <= 0:
        return None
    k = rng.choice(len(tri), 40000, p=ar / ar.sum())
    r1, r2 = rng.random(40000), rng.random(40000)
    fl = r1 + r2 > 1
    r1[fl], r2[fl] = 1 - r1[fl], 1 - r2[fl]
    sp = A[k] + (B[k] - A[k]) * r1[:, None] + (C[k] - A[k]) * r2[:, None]
    h = (sp - c) @ ax
    perp = (sp - c) - np.outer(h, ax)
    L = np.ptp(h)
    D = 2 * np.percentile(np.linalg.norm(perp, axis=1), 99)
    if L < 2.0 * D or D < 4e-3 or D > 60e-3:
        return None
    u = ev[:, 0]
    v = np.cross(ax, u)
    res = []
    for f in (0.25, 0.5, 0.75):
        sel = np.abs(h - (h.min() + f * L)) < max(0.02 * L, 1e-3)
        if sel.sum() < 12:
            return None
        p = perp[sel] - perp[sel].mean(0)
        th = np.arctan2(p @ v, p @ u)
        rr = np.linalg.norm(p, axis=1)
        b = ((th + math.pi) / (2 * math.pi) * 72).astype(int) % 72
        bins = np.full(72, np.nan)
        np.fmax.at(bins, b, rr)
        ok = ~np.isnan(bins)
        if ok.sum() < 54:
            return None
        ang = -math.pi + (np.arange(72)[ok] + 0.5) * math.pi / 36
        val = bins[ok]
        hm = {n: 2 * abs(np.mean(val * np.exp(-1j * n * ang))) for n in range(1, 9)}
        res.append((hm[6], max(hm[n] for n in (2, 3, 4, 5, 7, 8)), float(val.mean())))
    return min(x[0] for x in res), max(x[1] for x in res), float(np.mean([x[2] for x in res]))


def arrays(me):
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    me.calc_loop_triangles()
    tri = np.empty(len(me.loop_triangles) * 3, int)
    me.loop_triangles.foreach_get("vertices", tri)
    return co.reshape(-1, 3), tri.reshape(-1, 3)


names = {o.name for o in bpy.context.scene.objects if o.type == 'MESH'}
with bpy.data.libraries.load(CAD, link=False) as (src, dst):
    req = [n for n in src.objects if n in names]
    dst.objects = list(req)
dg = bpy.context.evaluated_depsgraph_get()
rng = np.random.default_rng(0)
seen, out = {}, []
for n, cob in zip(req, dst.objects):
    if cob is None or cob.type != 'MESH':
        continue
    if cob.data.name not in seen:
        co, tri = arrays(cob.data)
        seen[cob.data.name] = harmonics(co, tri, rng) if len(tri) else None
    hc = seen[cob.data.name]
    if not hc or not (hc[0] > 0.15e-3 and hc[0] > 2 * hc[1]):
        continue
    o = bpy.data.objects[n]
    me = o.evaluated_get(dg).to_mesh()
    co, tri = arrays(me)
    ho = harmonics(co, tri, rng) if len(tri) else None
    o.evaluated_get(dg).to_mesh_clear()
    a6o = ho[0] if ho else 0.0
    review = next((c.name for c in o.users_collection if c.name.startswith("REVIEW")), "")
    row = {"object": n, "cad_mesh": cob.data.name, "cad_a6_mm": round(hc[0] * 1e3, 3), "out_a6_mm": round(a6o * 1e3, 3),
           "R_mm": round(hc[2] * 1e3, 2), "ok": bool(a6o > 0.5 * hc[0]), "review": review}
    out.append(row)
    print("HEXCHECK", "ok   " if row["ok"] else "ROUND", f'{n[:50]:50s}', row["cad_mesh"], "a6 CAD", row["cad_a6_mm"], "out", row["out_a6_mm"],
          review, flush=True)
for ob in dst.objects:
    if ob is not None:
        bpy.data.objects.remove(ob)
json.dump(out, open(OUT, "w"), indent=1)
print("HEXCHECK", len(out), "hex stock objects,", sum(not r["ok"] for r in out), "came out round", flush=True)
