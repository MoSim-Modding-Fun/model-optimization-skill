# Limelight 3A library asset. The 3A (measured in a robot CAD: 81.0 x 49.0 x 29.5 mm incl. lips) is the Limelight 3G case:
# 26.0 mm case, lens barrel r 7.0 mm to 29.46 mm at (0, 8.875) mm, glass r 2.7 mm, the same +X power connector and -Y / +Y
# ports, but NO green LEDs, a light grey case ("Plastic - Grey 50") and a blue status LED in the 3G's +Y button hole.
# Built like limelight3g_asset.py (the LL4's own case body + corner-hole cutter, openings as boolean cutters).
# Frame as the LL4 / 3G: case centred in XY, back face at z=0, lens facing +Z. Build it in place (relative links to
# RobotMaterials): <out.blend> = "<library>/Components/Limelight 3A/Limelight 3A.blend".
#   blender -b --factory-startup --python limelight3a_asset.py -- <out.blend> [<robot CAD .blend> <3A camera object>]
# The optional CAD object goes into the HP collection, moved into this frame (assumes the CAD camera's local axes: lens
# along -Y, back face at +Y, long side X, the button side +Z; check the HP overlay after a build).
import bpy, bmesh, sys, os, math
from mathutils import Vector, Matrix

_a = sys.argv[sys.argv.index("--") + 1:]
OUT = _a[0]
CADSRC, CADOBJ = (_a[1], _a[2]) if len(_a) > 2 else (None, None)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import paths  # noqa: E402
LIB = paths.asset_library()
RM = f"{LIB}/Materials/RobotMaterials.blend"
LL4 = f"{LIB}/Components/Limelight 4/Limelight 4.blend"
LL3 = f"{LIB}/Components/Limelight 3/Limelight 3.blend"
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"  # same catalog as Limelight 3 / 4
MM = 0.001
FRONT = 26.0 * MM

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
bpy.context.preferences.filepaths.save_version = 0


def coll(name):
    c = bpy.data.collections.new(name)
    sc.collection.children.link(c)
    return c


C_ASSET, C_LP, C_CUT, C_HP = coll("Asset"), coll("LP"), coll("Cutters"), coll("HP")

with bpy.data.libraries.load(RM, link=True, relative=True) as (src, dst):
    dst.materials = ["Plastic - Black", "Camera Lens", "Plastic - Grey 50", "Plastic - Blue"]
BLACK, LENS, GREY, BLUE = (bpy.data.materials[n] for n in ("Plastic - Black", "Camera Lens", "Plastic - Grey 50", "Plastic - Blue"))

# ---------------------------------------------------------------- LL4 case body + corner-hole cutter (appended as is)
with bpy.data.libraries.load(LL4, link=False) as (src, dst):
    dst.objects = ["Cube", "Cube.001"]
case, corner = dst.objects
case.name, corner.name = "Case", "Corner Hole Cutter"
C_LP.objects.link(case)
C_CUT.objects.link(corner)
corner.hide_render = True
corner.display_type = 'WIRE'
case.data.materials.clear()
case.data.materials.append(GREY)  # 3A: light grey case (CAD 0.91)
SBA = next(m for m in case.modifiers if m.type == 'NODES').node_group


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
    else:
        for p in me.polygons:
            p.use_smooth = True
    return o


def revolve(bm, prof, centre, seg, mat_of=lambda i: 0, cap_top=True):
    """Lathe a (r, z) profile (metres) about the Z axis through centre; returns the rings. A zero radius closes a pole."""
    rings = []
    for r, z in prof:
        if r == 0:
            rings.append([bm.verts.new((centre.x, centre.y, z))])
        else:
            rings.append([bm.verts.new((centre.x + r * math.cos(2 * math.pi * k / seg), centre.y + r * math.sin(2 * math.pi * k / seg), z))
                          for k in range(seg)])
    for i in range(len(rings) - 1):
        a, b = rings[i], rings[i + 1]
        for k in range(seg):
            if len(a) == 1:
                f = bm.faces.new((a[0], b[k], b[(k + 1) % seg]))
            elif len(b) == 1:
                f = bm.faces.new((a[k], a[(k + 1) % seg], b[0]))
            else:
                f = bm.faces.new((a[k], a[(k + 1) % seg], b[(k + 1) % seg], b[k]))
            f.material_index = mat_of(i)
    return rings


def box(bm, lo, hi):
    bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Translation((Vector(lo) + Vector(hi)) / 2) @
                          Matrix.Diagonal((*(Vector(hi) - Vector(lo)), 1.0)))


# ---------------------------------------------------------------- case cutters: LED countersinks + side ports
cut = bmesh.new()
# (the 3A has no green LEDs: no countersinks)
W, Y = 40.055 * MM, 24.055 * MM
box(cut, (13.74 * MM, -Y - 1 * MM, 10.2 * MM), (29.54 * MM, -Y + 3.0 * MM, 23.6 * MM))      # RJ45, -Y side
box(cut, (W - 3.0 * MM, 7.15 * MM, 10.0 * MM), (W + 1 * MM, 14.95 * MM, 18.4 * MM))          # power connector, +X end
usb = bmesh.new()  # USB-C: 8.9 x 3.3 mm stadium, 2 mm deep, +Y side
outline = []
for k in range(16):
    a = math.pi / 2 + 2 * math.pi * k / 16
    cx = (8.9 - 3.3) / 2 * (1 if math.cos(a) > 1e-6 else -1 if math.cos(a) < -1e-6 else 0)
    outline.append(Vector(((13.64 + cx + 1.65 * math.cos(a)) * MM, 0, (11.4 + 1.65 * math.sin(a)) * MM)))
for y0, y1 in [(Y - 2.0 * MM, Y + 1 * MM)]:
    vs0 = [cut.verts.new((p.x, y0, p.z)) for p in outline]
    vs1 = [cut.verts.new((p.x, y1, p.z)) for p in outline]
    cut.faces.new(vs0[::-1]); cut.faces.new(vs1)
    for k in range(16):
        cut.faces.new((vs0[k], vs0[(k + 1) % 16], vs1[(k + 1) % 16], vs1[k]))
usb.free()
btn = Matrix.Translation(Vector((-25.86, 0, 13.8)) * MM) @ Matrix.Rotation(math.pi / 2, 4, 'X')  # 4 mm hole, 1 mm deep, +Y side
bmesh.ops.create_cone(cut, cap_ends=True, segments=12, radius1=2.0 * MM, radius2=2.0 * MM, depth=4.0 * MM,
                      matrix=Matrix.Translation((0, Y, 0)) @ btn)
bmesh.ops.recalc_face_normals(cut, faces=cut.faces)
cutters = obj("Front + Port Cutters", cut, [], C_CUT, cutter=True)

b = case.modifiers.new("Front + Ports", 'BOOLEAN')
b.operation, b.operand_type, b.object, b.solver = 'DIFFERENCE', 'OBJECT', cutters, 'EXACT'
for m in case.modifiers:
    if m.type == 'BOOLEAN':
        m.solver = 'EXACT'
case.modifiers.move(len(case.modifiers) - 1, 1)  # keep Smooth by Angle last

# ---------------------------------------------------------------- lens barrel (black) + glass, 32 sides like the LL4 lens
lb = bmesh.new()
L0 = Vector((0, 8.875, 0)) * MM
revolve(lb, [(7.0 * MM, FRONT - 0.3 * MM), (7.0 * MM, 29.46 * MM), (6.2 * MM, 29.46 * MM), (5.2 * MM, 28.65 * MM),
             (0, 28.65 * MM)], L0, 32, mat_of=lambda i: 1 if i == 3 else 0)
lens = obj("Lens", lb, [BLACK, LENS], C_LP)
s = lens.modifiers.new("Smooth by Angle", 'NODES'); s.node_group = SBA
w = lens.modifiers.new("WeightedNormal", 'WEIGHTED_NORMAL'); w.mode, w.weight, w.keep_sharp = 'FACE_AREA', 100, True

# ---------------------------------------------------------------- blue status LED in the +Y button hole (CAD: x -25.79 mm,
# r 1.75 mm, flush with the side): a 12-sided plug from the hole floor to 0.3 mm below the case side
lb = bmesh.new()
bmesh.ops.create_cone(lb, cap_ends=True, segments=12, radius1=1.95 * MM, radius2=1.95 * MM, depth=1.7 * MM,  # hole floor (Y - 2 mm) to Y - 0.3 mm
                      matrix=Matrix.Translation((0, Y - 1.15 * MM, 0)) @ btn)
leds = obj("LEDs", lb, [BLUE], C_LP)
s = leds.modifiers.new("Smooth by Angle", 'NODES'); s.node_group = SBA

# ---------------------------------------------------------------- asset object: applied + joined LP parts
dg = bpy.context.evaluated_depsgraph_get()
parts = []
for o in (case, lens, leds):
    me = bpy.data.meshes.new_from_object(o.evaluated_get(dg), preserve_all_data_layers=True, depsgraph=dg)
    t = bpy.data.objects.new(o.name + " (applied)", me)
    C_ASSET.objects.link(t)
    parts.append(t)
for t in parts:
    t.select_set(True)
bpy.context.view_layer.objects.active = parts[0]
with bpy.context.temp_override(active_object=parts[0], selected_editable_objects=parts, selected_objects=parts, object=parts[0]):
    bpy.ops.object.join()
asset = parts[0]
asset.name = asset.data.name = "Limelight 3A"
used = sorted({p.material_index for p in asset.data.polygons})  # booleans can leave an empty slot behind
remap = {old: new for new, old in enumerate(used)}
mats = [asset.data.materials[i] for i in used]
idx = [remap[p.material_index] for p in asset.data.polygons]
asset.data.materials.clear()  # (resets the face indices, so they are written back afterwards)
for m in mats:
    asset.data.materials.append(m)
asset.data.polygons.foreach_set("material_index", idx)
asset.data.update()
asset.asset_mark()
asset.asset_data.catalog_id = CATALOG
asset.asset_data.description = "Limelight 3A (Limelight 3G case, no LEDs, light grey), modelled like the Limelight 3G / 4 assets"
for t in ("camera", "vision", "electronics"):
    asset.asset_data.tags.new(t)

# ---------------------------------------------------------------- HP: the LL3G CAD, moved into this frame
hp = None
if CADSRC:
    with bpy.data.libraries.load(CADSRC, link=False) as (src, dst):
        dst.objects = [CADOBJ]
    hp = dst.objects[0]
if hp is not None:
    hp.name = "LIMELIGHT 3A (CAD)"
    hp.parent = None
    C_HP.objects.link(hp)
    co = [v.co for v in hp.data.vertices]
    cen = Vector([(min(v[k] for v in co) + max(v[k] for v in co)) / 2 for k in range(3)])
    R = Matrix(((1, 0, 0, 0), (0, 0, 1, 0), (0, -1, 0, 0), (0, 0, 0, 1)))
    hp.matrix_world = Matrix.Translation((0, 0, 14.73 * MM)) @ R @ Matrix.Translation(-cen)

for lc in bpy.context.view_layer.layer_collection.children:
    lc.hide_viewport = lc.name != "Asset"
tris = sum(len(p.vertices) - 2 for p in asset.data.polygons)
print("LL3A asset tris", tris, "polys", len(asset.data.polygons), "mats", [m.name if m else None for m in asset.data.materials])
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
for lib in list(bpy.data.libraries):  # appended sources (robot CAD, LL4) leave empty library entries: only RobotMaterials stays
    if not any(getattr(i, "library", None) == lib for c in (bpy.data.materials, bpy.data.objects, bpy.data.meshes, bpy.data.node_groups) for i in c):
        bpy.data.libraries.remove(lib)
bpy.ops.wm.save_as_mainfile(filepath=OUT, relative_remap=True)
print("LL3A saved", OUT, [l.filepath for l in bpy.data.libraries])
