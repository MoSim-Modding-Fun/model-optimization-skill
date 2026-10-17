# Which swerve pieces go into the library asset, following the MK4i asset: structure, drivetrain and wheel as
# primitives + the screws you notice; no washers, small bearings, balls, sensor (encoder) or tiny hardware.
#   python3 swerve_select.py <swerve work dir>   -> select.json
import json, sys, os
D = sys.argv[1]
rep = {x["object"]: x for x in json.load(open(os.path.join(D, "report.json")))["rows"]}
P = {p["name"]: p for p in json.load(open(os.path.join(D, "pieces.json")))["pieces"]}
px = json.load(open(os.path.join(D, "vis.json")))["passes"]["full"]["px"]
motors = json.load(open(os.path.join(D, "motors.json")))
STEER = (-61.0, -61.0)          # steering axis (mm, module frame)
BOTTOM_PLATE_Z = -2.0           # below this: the steering/wheel assembly ("Wheel" group, as on the MK4i)
SCREW_MIN_PX = 400              # screw heads smaller than this (max pixels over 80 views at 0.23 mm/px) are not noticeable
out = {}
for n, p in P.items():
    r = rep.get(n, {})
    e = sorted(p["ext_mm"]); x, y, z = p["ctr_mm"]; v = px.get(n, [0])[0]; mats = " ".join(p["mats"])
    meth = r.get("method", "")
    rsteer = ((x - STEER[0]) ** 2 + (y - STEER[1]) ** 2) ** 0.5
    if r.get("review"):
        cat, inc, why = "hidden", False, r["review"]
    elif n in motors:
        cat, inc, why = "motor", True, "library " + motors[n]["kind"]
    elif meth == "sphere" or (e[2] < 4 and abs(e[0] - e[2]) < 0.6):
        cat, inc, why = "ball", False, "bearing ball"
    elif rsteer < 12 and z > 55 and e[2] < 30:
        cat, inc, why = "sensor", False, "encoder on the steering axis (MK4i leaves out its CANcoder)"
    elif "mattesteel.003" in mats or (e[0] <= 5.5 and e[2] <= 24 and "defaultplastic" in mats and e[2] / max(e[0], 0.1) > 2.5):
        cat, inc, why = "bearing", False, "small bearing / shield"
    elif e[2] <= 10 and e[0] <= 6.5 and "mattesteel.001" in mats:
        cat, inc, why = "lock washer", False, "lock washer"
    elif e[0] <= 1.5 and e[2] <= 24:
        cat, inc, why = "washer", False, "thin washer/shim"
    elif "PlasticPolystyrene" in mats and e[0] <= 4.5:
        cat, inc, why = "spacer stack", True, "spacer ring (merged into one tube per stack)"
    elif ("mattesteel.001" in mats or "satinfinishstain" in mats) and e[1] <= 21 and e[2] <= 75:
        cat = "screw"
        inc = v >= SCREW_MIN_PX
        why = f"screw, {v} px" + ("" if inc else " (not noticeable)")
    elif e[2] < 8:
        cat, inc, why = "tiny", False, f"tiny part {e}"
    elif v < 300:
        cat, inc, why = "unnoticeable", False, f"{v} px"
    else:
        cat, inc, why = "part", True, meth
    out[n] = {"cat": cat, "include": inc, "why": why, "group": "Wheel" if z < BOTTOM_PLATE_Z and cat != "motor" else "Parts",
              "tris": r.get("after"), "px": v, "ext": e, "ctr": p["ctr_mm"]}
json.dump(out, open(os.path.join(D, "select.json"), "w"), indent=0)
import collections
c = collections.defaultdict(lambda: [0, 0, 0])
for n, o in out.items():
    k = (o["cat"], o["include"]); c[k][0] += 1; c[k][1] += o["tris"] or 0
for (cat, inc), (k, t, _) in sorted(c.items(), key=lambda kv: (not kv[0][1], kv[0][0])):
    print(f"{'IN ' if inc else 'out'} {cat:14s} {k:3d} pieces {t:>7,} tris  e.g. {[n[-3:] for n, o in out.items() if o['cat'] == cat and o['include'] == inc][:14]}")
print("included tris (motors as CAD):", sum(o["tris"] or 0 for o in out.values() if o["include"]))
