# Limelight 2+ library asset, modelled like the Limelight 4 / 3G assets. Owner (2026-10-14): "limelight's should
# be the box, the lens, and the lights visible on the front, and any details visible on the front" -> case box (rounded
# corners concentric with the 4 corner through-holes, 1.7 mm chamfer round the front and back faces), the 8 LED recesses
# with lit floors, the lens (barrel ring + inner ring + glass); side ports, buttons and labels left out.
# Measured from the vendor-simplified LL2+ CAD ("id32", 97.9 x 57.0 x 25.0 mm case): front face 94.5 x 53.2 mm,
# corner radius 4.49 mm about the hole centres (+-44.45, +-23.81), holes r 2.4 mm; LEDs r 3.67 mm, 0.51 mm deep, at
# (+-19, +-8.5), (+-20, 0), (+-28.75, 0) from the lens; lens 9.5 mm off the case centre along the short side.
# Frame as the LL4: case centred in XY, back face at z = 0, front (lens) facing +Z. HP = the CAD, moved into this frame.
#   blender -b <robot with the CAD>.blend --python limelight2_asset.py -- <cad object> <out.blend>
import bpy, bmesh, sys, os, math
from mathutils import Vector, Matrix

CAD_NAME, OUT = sys.argv[sys.argv.index("--") + 1:][:2]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import paths  # noqa: E402
LIB = paths.asset_library()
RM = f"{LIB}/Materials/RobotMaterials.blend"
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"  # same catalog as Limelight 3 / 3G / 4
MM = 0.001
DEPTH = 25.0 * MM          # back face z = 0, front face z = DEPTH
HX, HY = 44.45 * MM, 23.81 * MM  # corner-hole centres = corner-arc centres
RC = 4.49 * MM             # case corner radius
CH = 1.71 * MM             # chamfer round the front and back faces (front outline 94.47 x 53.19 mm in the CAD)
RH = 2.4 * MM              # corner through-holes (r 2.61 at the faces, 2.2 mid)
LENS_Y = 9.5 * MM          # lens centre, from the case centre (CAD: lens at y 0, case centre at y 9.5 -> flipped)
LEDS = [(x, y) for x, y in ((19.0, 8.5), (19.0, -8.5), (20.0, 0.0), (28.75, 0.0))]
LEDS = [Vector((sx * x, y, 0)) * MM + Vector((0, LENS_Y, 0)) for x, y in LEDS for sx in (1, -1)]
R_LED, LED_DEPTH = 3.67 * MM, 0.51 * MM
CAD_SIDES = 8  # owner's side tiers: r < 6 mm -> 8

# CAD (id32 local, front at z 0.36, back at 25.36, case centre y 9.5) -> this frame: 180 deg about X, back face to z = 0
CAD_TO_ASSET = Matrix.Translation((0, 0, 25.36 * MM)) @ Matrix.Rotation(math.pi, 4, 'X') @ Matrix.Translation((0, -9.5 * MM, 0))

cad_me = bpy.data.objects[CAD_NAME].data.copy()  # keep the CAD mesh (and its materials) for HP, drop the robot
sc = bpy.context.scene
for o in list(bpy.data.objects):
    bpy.data.objects.remove(o)
for c in list(bpy.data.collections):
    bpy.data.collections.remove(c)
for w in list(bpy.data.worlds) + list(bpy.data.cameras):
    pass
bpy.context.preferences.filepaths.save_version = 0


def coll(name):
    c = bpy.data.collections.new(name)
    sc.collection.children.link(c)
    return c


C_ASSET, C_LP, C_CUT, C_HP = coll("Asset"), coll("LP"), coll("Cutters"), coll("HP")
with bpy.data.libraries.load(RM, link=True, relative=True) as (src, dst):
    dst.materials = ["Plastic - Black", "Camera Lens", "LED - Lime"]
BLACK, LENS, LED = (bpy.data.materials[n] for n in ("Plastic - Black", "Camera Lens", "LED - Lime"))


def obj(name, bm, mats, collection, cutter=False):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    for m in mats:
        me.materials.append(m)
    o = bpy.data.objects.new(name, me)
    collection.objects.link(o)
    if cutter:
        o.hide_render = True
        o.display_type = 'WIRE'
    return o


def outline(inset, seg_per_quarter=2):
    """Rounded-rectangle outline (corner arcs about the hole centres), counter-clockwise, radius RC - inset."""
    pts = []
    for cx, cy, a0 in ((HX, HY, 0), (-HX, HY, 90), (-HX, -HY, 180), (HX, -HY, 270)):
        for k in range(seg_per_quarter + 1):
            a = math.radians(a0 + 90 * k / seg_per_quarter)
            pts.append(Vector((cx + (RC - inset) * math.cos(a), cy + (RC - inset) * math.sin(a), 0)))
    return pts


# ---------------------------------------------------------------- case: chamfered rounded box
bm = bmesh.new()
rings = []
for inset, z in ((CH, 0.0), (0.0, CH), (0.0, DEPTH - CH), (CH, DEPTH)):
    rings.append([bm.verts.new((p.x, p.y, z)) for p in outline(inset)])
n = len(rings[0])
for a, b in zip(rings, rings[1:]):
    for k in range(n):
        bm.faces.new((a[k], a[(k + 1) % n], b[(k + 1) % n], b[k]))
bm.faces.new(rings[0][::-1])
bm.faces.new(rings[-1])
bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
case = obj("Case", bm, [BLACK], C_LP)
for p in case.data.polygons:
    p.use_smooth = False

# ---------------------------------------------------------------- cutters: corner through-holes + LED recesses
cut = bmesh.new()
for sx in (1, -1):
    for sy in (1, -1):
        bmesh.ops.create_cone(cut, cap_ends=True, segments=CAD_SIDES, radius1=RH, radius2=RH, depth=DEPTH + 2 * MM,
                              matrix=Matrix.Translation((sx * HX, sy * HY, DEPTH / 2)) @ Matrix.Rotation(math.pi / CAD_SIDES, 4, 'Z'))
for c in LEDS:
    bmesh.ops.create_cone(cut, cap_ends=True, segments=CAD_SIDES, radius1=R_LED, radius2=R_LED, depth=2 * LED_DEPTH,
                          matrix=Matrix.Translation((c.x, c.y, DEPTH)) @ Matrix.Rotation(math.pi / CAD_SIDES, 4, 'Z'))
bmesh.ops.recalc_face_normals(cut, faces=cut.faces)
cutters = obj("Hole + LED Cutters", cut, [], C_CUT, cutter=True)
b = case.modifiers.new("Holes + LEDs", 'BOOLEAN')
b.operation, b.operand_type, b.object, b.solver = 'DIFFERENCE', 'OBJECT', cutters, 'EXACT'

# ---------------------------------------------------------------- lens: barrel ring + inner ring (black), glass, 16 sides (r 7 mm tier)
# (r, z above the front face) from the CAD: ring r 7.0 rising 0.76 mm, sloping to r 3.95; inner ring at r 2.7; glass
PROF = [(7.0, 0.0), (7.0, 0.55), (6.2, 0.76), (3.95, 0.32), (2.7, 0.26), (2.7, 0.36), (0.0, 0.36)]
lb = bmesh.new()
L0 = Vector((0, LENS_Y, DEPTH))
seg = 16
rings = []
for r, z in PROF:
    if r == 0:
        rings.append([lb.verts.new(L0 + Vector((0, 0, z * MM)))])
    else:
        rings.append([lb.verts.new(L0 + Vector((r * MM * math.cos(2 * math.pi * k / seg), r * MM * math.sin(2 * math.pi * k / seg), z * MM)))
                      for k in range(seg)])
for i in range(len(rings) - 1):
    a, c = rings[i], rings[i + 1]
    for k in range(seg):
        if len(c) == 1:
            f = lb.faces.new((a[k], a[(k + 1) % seg], c[0]))
        else:
            f = lb.faces.new((a[k], a[(k + 1) % seg], c[(k + 1) % seg], c[k]))
        f.material_index = 1 if i == len(rings) - 2 else 0  # glass = the last (r 2.7 -> 0) cap
bmesh.ops.recalc_face_normals(lb, faces=lb.faces)
for f in lb.faces:  # open ring standing on the front face: every face must point away from the case
    if f.normal.z < -0.5 or (f.calc_center_median() - L0).xy.dot(f.normal.xy) < -1e-9 and abs(f.normal.z) < 0.5:
        f.normal_flip()
lens = obj("Lens", lb, [BLACK, LENS], C_LP)

# ---------------------------------------------------------------- asset object: case applied + LED floors lit + lens
dg = bpy.context.evaluated_depsgraph_get()
me = bpy.data.meshes.new_from_object(case.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
me.materials.clear()
for m in (BLACK, LENS, LED):
    me.materials.append(m)
bm = bmesh.new(); bm.from_mesh(me)
lit = 0
for f in bm.faces:
    f.material_index = 0
    c = f.calc_center_median()
    if f.normal.z > 0.99 and abs(c.z - (DEPTH - LED_DEPTH)) < 1e-5 and any((c.xy - l.xy).length < R_LED for l in LEDS):
        f.material_index = 2
        lit += 1
lm = lens.data
off = len(bm.verts)
vs = [bm.verts.new(v.co) for v in lm.vertices]
for p in lm.polygons:
    f = bm.faces.new([vs[i] for i in p.vertices])
    f.material_index = p.material_index  # 0 black, 1 lens
bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
bm.to_mesh(me); bm.free()
for p in me.polygons:
    p.use_smooth = False
asset = bpy.data.objects.new("Limelight 2+", me)
me.name = "Limelight 2+"
C_ASSET.objects.link(asset)
asset.asset_mark()
asset.asset_data.catalog_id = CATALOG
asset.asset_data.description = ("Limelight 2+ (box, lens and the 8 front LEDs; built from a vendor CAD, Limelight 4 style)")
for t in ("camera", "vision", "electronics"):
    asset.asset_data.tags.new(t)

# ---------------------------------------------------------------- HP: the CAD, moved into this frame
hp = bpy.data.objects.new("LIMELIGHT2+ (CAD)", cad_me)
C_HP.objects.link(hp)
hp.matrix_world = CAD_TO_ASSET
for lc in bpy.context.view_layer.layer_collection.children:
    lc.exclude = lc.name in ("HP",)
    lc.hide_viewport = lc.name != "Asset"

tris = sum(len(p.vertices) - 2 for p in me.polygons)
print("LL2 asset tris", tris, "polys", len(me.polygons), "lit LED floors", lit, "mats", [m.name for m in me.materials], flush=True)
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, relative_remap=True)
print("LL2 saved", OUT, [l.filepath for l in bpy.data.libraries], flush=True)
