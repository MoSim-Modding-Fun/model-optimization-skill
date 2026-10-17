# Limelight 3G library asset, built like the Limelight 4 asset: the LL4's own case body and corner-hole cutter (both
# Limelights share the 80.1 x 48.1 x 26 mm case and the +-36 x +-20 mm hole pattern), the 3G front (lens barrel, two green
# LEDs in countersunk recesses) and its side ports as boolean cutters, all live in LP/Cutters; the asset object
# "Limelight 3G" is the applied, joined result. Frame as the LL4: case centred in XY, back face at z=0, lens facing +Z.
# Dimensions measured from the LL3G CAD (kept in the HP collection, moved into this frame).
#   blender -b --factory-startup --python limelight3g_asset.py -- "<library>/Components/Limelight 3G/Limelight 3G.blend"
# Checked with the 26-direction culling gap check (0 see-through pixels); the asset-browser preview is a separate render.
import bpy, bmesh, sys, os, math
from mathutils import Vector, Matrix

OUT = sys.argv[sys.argv.index("--") + 1]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import paths  # noqa: E402
LIB = paths.asset_library()
RM = f"{LIB}/Materials/RobotMaterials.blend"
LL4 = f"{LIB}/Components/Limelight 4/Limelight 4.blend"
LL3 = f"{LIB}/Components/Limelight 3/Limelight 3.blend"
CATALOG = "95d0853d-fe06-4839-ab8e-92497fbc8b1a"  # same catalog as Limelight 3 / 4
CAD_OFFSET = Vector((-83.53, 21.2, 8.05)) / 1000  # LL3G CAD (as stored in Limelight 3.blend) -> this frame
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
    dst.materials = ["Plastic - Black", "Camera Lens", "LED - Lime"]
BLACK, LENS, LED = (bpy.data.materials[n] for n in ("Plastic - Black", "Camera Lens", "LED - Lime"))

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
case.data.materials.append(BLACK)
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
LEDS = [Vector((17.25, 14.38, 0)) * MM, Vector((17.25, 3.38, 0)) * MM]
for c in LEDS:  # 37 deg countersink: r 4.19 mm at the face to 1.7 mm, 3.5 mm deep, flat floor
    rings = revolve(cut, [(0, 22.5 * MM), (1.7 * MM, 22.5 * MM), (4.19 * MM + 0.6 * MM, FRONT + 0.8 * MM), (0, FRONT + 0.8 * MM)], c, 24)
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

# ---------------------------------------------------------------- green LEDs (domes sitting on the recess floors)
lb = bmesh.new()
for c in LEDS:
    revolve(lb, [(1.15 * MM, 22.45 * MM), (1.0 * MM, 23.25 * MM), (0.6 * MM, 23.62 * MM), (0, 23.74 * MM)], c, 12)
leds = obj("LEDs", lb, [LED], C_LP)
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
asset.name = asset.data.name = "Limelight 3G"
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
asset.asset_data.description = "Limelight 3G (global-shutter Limelight 3), modelled like the Limelight 4 asset"
for t in ("camera", "vision", "electronics"):
    asset.asset_data.tags.new(t)

# ---------------------------------------------------------------- HP: the LL3G CAD, moved into this frame
with bpy.data.libraries.load(LL3, link=False) as (src, dst):
    dst.objects = ["LIMELIGHT3GCAD_STEP.036"]
hp = dst.objects[0]
hp.name = "LIMELIGHT3G (CAD)"
C_HP.objects.link(hp)
hp.matrix_world = Matrix.Translation(CAD_OFFSET) @ hp.matrix_world

for lc in bpy.context.view_layer.layer_collection.children:
    lc.hide_viewport = lc.name != "Asset"
tris = sum(len(p.vertices) - 2 for p in asset.data.polygons)
print("LL3G asset tris", tris, "polys", len(asset.data.polygons), "mats", [m.name if m else None for m in asset.data.materials])
bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT, relative_remap=True)
print("LL3G saved", OUT, [l.filepath for l in bpy.data.libraries])
