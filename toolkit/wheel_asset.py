# Low-poly spoked wheel library assets (flex / compliant wheels) in the style of the team's wheel assets
# (WCP-1290, WCP-1362, am-3562): tread ring + spoke windows + hub with a 1/2" hex bore, one prism; optional core ring
# in a second finish. Axle = local Z, centred at the origin, no rotation. Variants share the shape (one per finish).
#   blender -b "<robot> - cleaned.blend" --python wheel_asset.py -- <spec name> <out.blend> <preview dir|-> [<CAD object>]
# The CAD object defaults to the spec's "cad" name; exporter instance suffixes ("-<n>-<n>", ".NNN") are matched.
# Specs below were measured on the source CADs (OD, width, ring/hub radii, spoke count and width).
import bpy, sys, os, math, re
import numpy as np
from mathutils import Vector, Matrix
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build, geo, libcombo, align
MM = 1e-3
import paths  # noqa: E402
RM = paths.robot_materials()
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"
HEX_APOTHEM = 6.35  # 1/2" hex
SPECS = {
    # 2"x1" WCP solid-core flex: big hub (~55% of OD), 6 round-cornered pockets r 14.5-21.3 mm, spokes ~6.5 mm
    "WCP-1296": dict(cad="Straight Flex Wheel-2-1-30A-1_2 Hex (WCP-1296)", od=50.8, w=25.4, ring_in=21.3, hub=14.5,
                     spokes=6, spoke_w=6.5, corner=2.0, core=None,
                     variants=[("WCP-1296 (2in OD 1in WD 30A Flex Wheel)", "Rubber - Black", None)]),
    # 2"x0.5" VEX flex: small hub around the hex, 6 thin spokes, large round-cornered windows
    "217-6353": dict(cad="2 OD Straight Flex Wheel (217-6353)", od=50.8, w=12.7, ring_in=20.7, hub=10.3,
                     spokes=6, spoke_w=4.6, corner=1.5, core=None, bore=5.49, hex_on_spokes=True,  # CAD hex: 5.49 mm apothem, corners on the spokes
                     variants=[("217-6353 (2in OD 0.5in WD 30A Flex Wheel)", "Rubber - Colson Wheel", None)]),
    "am-3945": dict(cad="3IN 500Hex Compliant Wheel REV 3 (am-3945 )", od=76.2, w=25.4, ring_in=33.5, hub=15.0,
                    spokes=9, spoke_w=4.2, corner=0.0, core=10.0,
                    variants=[("am-3945_green (3in OD 1in WD 35A Compliant Wheel)", "Rubber - Green", "Plastic - White"),
                              ("am-3945_blue (3in OD 1in WD 50A Compliant Wheel)", "Rubber - Blue", "Plastic - White")]),
    # 1.625"x0.5" 45A flex wheel: solid disc, 6 through arc slots r 13.9-16.6 mm with round ends, each 44.5 deg at
    # r 15.25 (spokes 4.1 mm), centred between the hex flats' normals (hex corners point at the slots, as in the CAD)
    "flex1625": dict(cad='1.625" Flex Wheel (45A, 1/2" Wide, 1/2" Hex Bore)__1.625" Flex Wheel (45A, 1/2" Wide, 1/2" Hex Bore)',
                     od=41.275, w=12.7, ring_in=16.6, hub=13.9, spokes=6, spoke_w=4.1, corner=1.3, core=None,
                     variants=[("1.625in Flex Wheel (1.625in OD 0.5in WD 45A Flex Wheel)", "Rubber - Black", None)]),
    # 1.625"x0.5" 30A flex wheel, 1/2" hex STRETCH bore: 6 spokes 4.0 mm wide, windows r 9.45-16.64 mm with
    # 1.5 mm corners (area 66.4 mm2 = CAD), undersized hex bore apothem 4.76 with its corners pointing at the windows.
    # Built from a robot CAD (judged CLOSE ENOUGH; library "1.625in Flex Wheel (... 30A Flex Wheel, Stretch Bore)")
    "flex1625s": dict(cad='1.625" Flex Wheel (30A, 1/2" Wide, 1/2" Hex Stretch Bore).007', od=41.275, w=12.7, ring_in=16.64,
                      hub=9.45, spokes=6, spoke_w=4.0, corner=1.5, core=None, bore=4.76,
                      variants=[("1.625in Flex Wheel (1.625in OD 0.5in WD 30A Flex Wheel, Stretch Bore)", "Rubber - Black", None)]),
    # 4"x1" Thrifty Squish Wheel 45A: one-piece, 6 rounded windows r 15.2-44.45 mm centred between the spokes,
    # spokes 6.6 mm, ~4 mm window corners, hex bore apothem 6.44 with its corners pointing at the windows (CAD section)
    "squish4": dict(cad='4" Thrifty Squish Wheel (45A, 1" Wide, 1/2" Hex Bore)', od=101.6, w=25.4, ring_in=44.45, hub=15.2,
                    spokes=6, spoke_w=6.6, corner=4.0, core=None, bore=6.44,
                    variants=[("4in Squish Wheel (4in OD 1in WD 45A Thrifty Squish Wheel)", "Rubber - Black", None)]),
    # 3"x1" Thrifty Squish Wheel 45A: one-piece, 5 rounded windows r 13.6-32.9 mm, spokes 5.2 -> 4.7 mm (5.0),
    # ~4.5 mm corners, hex bore apothem 6.44 (mid-width section, ray parity per 1 mm radius); 616 tris, fit 1.5 / 1.9 mm
    "squish3": dict(cad='3" Thrifty Squish Wheel (45A, 1" Wide, 1/2" Hex Bore)', od=76.2, w=25.4, ring_in=32.9, hub=13.6,
                    spokes=5, spoke_w=5.0, corner=4.5, core=None, bore=6.44,
                    variants=[("3in Squish Wheel (3in OD 1in WD 45A Thrifty Squish Wheel)", "Rubber - Black", None)]),
}
SPEC, OUT, PREVIEW_DIR = sys.argv[sys.argv.index("--") + 1:][:3]
CAD_NAME = sys.argv[sys.argv.index("--") + 4] if len(sys.argv) > sys.argv.index("--") + 4 else None
P = SPECS[SPEC]
X, Y, Z = Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1))


def circle(r, n, phase=0.0):
    return np.array([(r * math.cos(phase + 2 * math.pi * k / n), r * math.sin(phase + 2 * math.pi * k / n)) for k in range(n)])


def hexagon(apothem, corners_on_spokes=False):
    """Hex bore; by default the flats face the spokes (corners at 30 deg), else corners point at the spokes (0 deg)."""
    R = apothem / math.cos(math.pi / 6)
    return circle(R, 6, 0.0 if corners_on_spokes else math.pi / 6)


def round_corner(prev, v, nxt, rc, k=0.41):
    """Replace a sharp corner by a 3-point fillet approximation (two tangent points + a point pulled toward the corner)."""
    a = v + (prev - v) / np.linalg.norm(prev - v) * rc
    b = v + (nxt - v) / np.linalg.norm(nxt - v) * rc
    m = (a + b) / 2
    return [a, m + (v - m) * k, b]


def window(a0, a1, r0, r1, step):
    """Annular-sector hole between two spokes: outer arc a0->a1 at r1, inner arc back at r0 (spoke edges stay parallel),
    corners rounded with radius P["corner"] (mm) when set. Each arc is trimmed by the fillet length before the
    3-point fillet goes in: on the short inner arc the old per-vertex fillet overshot the next arc vertex (jagged slivers
    where the spokes meet the hub)."""
    hw = P["spoke_w"] / 2 * MM
    ao0, ao1 = a0 + hw / r1, a1 - hw / r1          # outer arc ends
    ai0, ai1 = a0 + hw / r0, a1 - hw / r0          # inner arc ends
    rc = P.get("corner", 0.0) * MM
    pol = lambda r, a: np.array((r * math.cos(a), r * math.sin(a)))
    if rc <= 0:
        m = max(2, int(math.ceil((ao1 - ao0) / step)) + 1)
        mi = max(2, int(math.ceil((ai1 - ai0) / step)) + 1)
        return np.array([pol(r1, a) for a in np.linspace(ao0, ao1, m)] + [pol(r0, a) for a in np.linspace(ai1, ai0, mi)])
    dto = min(rc / r1, 0.45 * (ao1 - ao0))
    dti = min(rc / r0, 0.45 * (ai1 - ai0))
    m = max(2, int(math.ceil((ao1 - ao0 - 2 * dto) / step)) + 1)
    mi = max(2, int(math.ceil((ai1 - ai0 - 2 * dti) / step)) + 1)
    outer = [pol(r1, a) for a in np.linspace(ao0 + dto, ao1 - dto, m)]
    inner = [pol(r0, a) for a in np.linspace(ai1 - dti, ai0 + dti, mi)]
    C1, C2, C3, C4 = pol(r1, ao1), pol(r0, ai1), pol(r0, ai0), pol(r1, ao0)
    e12 = (C2 - C1) / np.linalg.norm(C2 - C1)
    e34 = (C4 - C3) / np.linalg.norm(C4 - C3)
    rs = min(rc, 0.45 * np.linalg.norm(C2 - C1))

    def fillet(a, c, b, k=0.41):
        mid = (a + b) / 2
        return mid + (c - mid) * k

    pts = outer + [fillet(outer[-1], C1, C1 + e12 * rs), C1 + e12 * rs, C2 - e12 * rs, fillet(C2 - e12 * rs, C2, inner[0])]
    pts += inner + [fillet(inner[-1], C3, C3 + e34 * rs), C3 + e34 * rs, C4 - e34 * rs, fillet(C4 - e34 * rs, C4, outer[0])]
    return np.array(pts)


R = P["od"] / 2 * MM
n_out = geo.ngon_segments(P["od"] / 2)
step = 2 * math.pi / n_out
W = P["w"] * MM
holes = []
for k in range(P["spokes"]):
    a0 = 2 * math.pi * k / P["spokes"]
    holes.append(window(a0, a0 + 2 * math.pi / P["spokes"], P["hub"] * MM, P["ring_in"] * MM, step))
if P["core"]:
    holes.append(circle(P["core"] * MM, geo.ngon_segments(P["core"])))
else:
    holes.append(hexagon(P.get("bore", HEX_APOTHEM) * MM, P.get("hex_on_spokes", False)))
body = build.build_prism("body", circle(R, n_out), holes, X, Y, Z, -W / 2, W / 2)
core = build.build_prism("core", circle(P["core"] * MM, geo.ngon_segments(P["core"])), [hexagon(HEX_APOTHEM * MM)], X, Y, Z, -W / 2, W / 2) if P["core"] else None

# fit check vs the CAD wheel (axis to Z, centred)
cad = bpy.data.objects.get(CAD_NAME or P["cad"]) or next(
    (o for o in bpy.data.objects if re.fullmatch(re.escape(P["cad"]) + r"(-\d+-\d+|\.\d{3})", o.name)), None)
if cad is None:
    raise SystemExit(f"CAD object {CAD_NAME or P['cad']!r} not found: pass its name as the 4th argument")
cco, ctri, _ = geo.mesh_arrays(cad.data)
k = int(np.argmin(np.ptp(cco, axis=0)))
order = [j for j in range(3) if j != k] + [k]
cco = cco[:, order] - (cco[:, order].min(0) + cco[:, order].max(0)) / 2
if np.linalg.det(np.eye(3)[order]) < 0:
    cco[:, 0] *= -1  # keep a right-handed frame
whole = libcombo.join_meshes("chk", [(m, Matrix.Identity(4)) for m in [body] + ([core] if core else [])])
aco, atri, _ = geo.mesh_arrays(whole)
A, _ = geo.sample_surface(aco, atri, 4000); B, _ = geo.sample_surface(cco, ctri, 15000)
ch = align.chamfer(A, B, np.eye(3), np.zeros(3))
bpy.data.meshes.remove(whole)
print("WHEEL", SPEC, "chamfer mm", [round(c * 1000, 2) for c in ch], flush=True)

# materials + variants
names = sorted({m for _, t, c in P["variants"] for m in (t, c) if m})
with bpy.data.libraries.load(RM, link=False) as (src, dst):
    dst.materials = names
MAT = {m.name: m for m in dst.materials}
meshes = []
for asset_name, tread, corem in P["variants"]:
    b = body.copy(); b.materials.clear(); b.materials.append(MAT[tread])
    parts = [(b, Matrix.Identity(4))]
    if core:
        c = core.copy(); c.materials.clear(); c.materials.append(MAT[corem]); parts.append((c, Matrix.Identity(4)))
    me = libcombo.join_meshes(asset_name, parts)
    for m, _ in parts:
        bpy.data.meshes.remove(m)
    me.name = asset_name
    meshes.append((asset_name, me))
    print("WHEEL", asset_name, "tris", sum(len(p.vertices) - 2 for p in me.polygons), flush=True)

# new file content: assets (+ the CAD reference, excluded from the view layer)
cad_ref = cad.data.copy()
for o in list(bpy.data.objects):
    bpy.data.objects.remove(o)
for c in list(bpy.data.collections):
    bpy.data.collections.remove(c)
sc = bpy.context.scene
assets = []
for i, (asset_name, me) in enumerate(meshes):
    ob = bpy.data.objects.new(asset_name, me)
    sc.collection.objects.link(ob)
    ob.location.x = i * 2.5 * R  # side by side in the file; each asset is centred on its own origin
    ob.asset_mark()
    ob.asset_data.catalog_id = CATALOG
    assets.append(ob)
ref = bpy.data.collections.new("CAD reference")
sc.collection.children.link(ref)
ref.objects.link(bpy.data.objects.new(P["cad"] + " (CAD)", cad_ref))
bpy.context.view_layer.layer_collection.children["CAD reference"].exclude = True
if PREVIEW_DIR != "-":
    for ob in assets:
        png = os.path.join(PREVIEW_DIR, ob.name + ".png")
        if os.path.exists(png):
            try:
                with bpy.context.temp_override(id=ob):
                    bpy.ops.ed.lib_id_load_custom_preview(filepath=png)
                print("WHEEL preview", ob.name, flush=True)
            except Exception as ex:
                print("WHEEL preview failed", ex, flush=True)
bpy.ops.outliner.orphans_purge(do_recursive=True)
bpy.context.preferences.filepaths.save_version = 0
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
print("WHEEL saved", OUT, flush=True)
