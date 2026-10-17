# Building library assets

How to build a new asset for the library (or for one robot) so it matches the existing ones. Read
[asset-library.md](asset-library.md) first for layout, naming and the git policy. `$TK` = the skill's `toolkit/`.

## Contents
1. Rules for every asset
2. Swerve and other modules
3. Compliant, flex and other wheels
4. Cameras and Limelights
5. Motors, batteries, signal lights and small electronics
6. Kit parts (stock channels, plates, blocks, servos)
7. Motor caps and other accessories

## 1. Rules for every asset

- **Check it doesn't exist first**: `ls Components | grep -i <part number or size>`, open the closest candidates and
  compare sizes, AND check with the robot sessions running now. Two sessions built the same wheel into the same folder the same
  night; the second overwrote the first.
- **Analyse the reference asset before modelling** (the existing module for modules, the existing compliant wheel for
  wheels, the motor assets for motors, the Limelight 4 for cameras): map its low-poly objects to its own `HP` CAD
  collection and render it next to the CAD. "Same degree of detail" means what the reference includes, not CAD
  fidelity.
- **Nothing hidden inside**: delete faces never seen with face-id renders (`"$TK/asset_prune.py"`: unique flat colour per
  face, ~100+ views with culling on), then a gap check (`"$TK/gapcheck.py"`: 26 directions, culling off vs on, 0 differing
  pixels). A 32-ray visibility test missed faces seen only through 0.5 mm slits. `asset_prune.py` encodes face ids in 15
  bits: fewer than 32,768 faces per collection run.
- **No phasing**: fitted pieces are clipped 0.25 mm inside neighbouring faces; check the sides in close-up, not only
  the iso view.
- **Watertight and outward**: recalc normals and fix inverted shells after Collapse; check with culling on (unculled
  renders hid inside-out output for several rounds).
- **Finishes** from `RobotMaterials.blend` by part type, library materials left as shipped; link them with
  `relative=True` and write the asset **straight into the library folder** (saving with `relative_remap`): a staging
  copy stores a long relative path that breaks when copied.
- **Metadata** like the neighbours: `asset_mark()`, the same catalog id as similar assets, tags, a description (what it
  is, tris), a 256 px custom preview (`bpy.ops.ed.lib_id_load_custom_preview` under `temp_override(id=…)`), the source
  CAD in an excluded `HP` collection, Z up, origin at a meaningful point (a mount plane), compressed save,
  `save_version = 0` (no `.blend1` in the library), refuse to overwrite an existing file.
- `mesh.materials.clear()` resets every face's material index: write the indices back. A boolean can add an empty
  material slot. `libraries.load(link=False)` leaves transient library entries that vanish on reload.
- Appended objects report an identity / zero `matrix_world` until they are linked into a scene and
  `view_layer.update()` has run; compose `matrix_basis` up the parents if needed. **Record world matrices before moving
  anything** in an extract script (a placed asset once landed 300 mm off).
- **Compare** with the CAD in renders from 4 views and get a no-context CLOSE ENOUGH verdict before using it in a robot.
- A rebuild with better code can come out heavier with no visible difference: keep the old file (`lib_backup/`) and
  compare tris and close-ups before replacing.

## 2. Swerve and other modules

Module assets are **motorless** (the robot keeps separate library motors, `LIB_IDENTITY_MOTORS`), contain **nothing
hidden inside the shell**, and carry the reference module's detail: plates (a pocketed top plate can be ~4k), standoffs
(8 sides), shafts, every gear/pulley as a plain disc, the belt, big bearings as flat rings, the wheel group (plain tread
band, hub, simple wheel mounts, axle bolt, bevel gears), and only the screws whose heads you notice (seen by ≥ 400 px in
the module render), no threads. Dropped even when visible: encoder and its housing/magnet, small bearings (< 300 px),
washers, clips, shims, bearing balls, tiny hardware. Decals become plain shapes. Finishes by part type (machined
aluminium plates/standoffs/shafts, steel gears/screws, polished pulley, black rubber tread); the robot then recolours
from its CAD per the colour rule.

Three starting points:
- **Vendor CAD with part names** (`mk4n_extract.py` + `mk4n_asset.py`, `x2i_*.py`): move each module type into the
  module frame, `vis.py` + `pipeline.py` with a module config (`robots/rmk4n.py`, `rx2i.py`: 1 µm pre-weld, plain tread
  band, motors kept as is), select by part name. Watch name traps ("Wheel, …, Bearing Bore" is the hub, not a bearing).
  Mirrored module types (A/B) get separate collections. The steering turret = the pieces whose pose relative to the
  base plate differs between the robot's modules.
- **Fused vendor export without names** (`mk5n_extract.py`, `mk5n_asset.py`, `robots/rmk5n.py`): split by islands, keep
  fused bodies whole and Collapse them (ratio by render-error knee, or the owner's cheaper pick; **match the reference
  asset's tri counts, not its ratio**: ratios depend on the export's density, 0.075 on a 236k housing ≈ 0.20 on a 31k
  one). Check shape routing of new pieces (a turret read as "round" became a puck → force `"decimate"`). Another
  revision of the module → its own library file (`MK5N_TYPES`, `MK5N_LIBFILE`).
- **Custom module from the robot's CAD** (`swerve_extract.py` → `swerve_pieces.py` → `swerve_prep.py` (fit library
  motors, keep their poses, drop them) → `vis.py` + `pipeline.py` with a module config (`ROUND_BY_SHAPE`, `SPHERES`,
  `KEEP_AS_IS` for motor pieces) → `swerve_select.py` → `swerve_asset.py`). `swerve_select.py`'s constants (steer axis,
  bottom-plate z, pixel thresholds, category rules) are per module; check its classification of every part.
- A robot whose module differs from the vendor's (taller, other standoffs, through-frame bolts, another wheel) gets a
  robot-kept asset next to the robot. Vendor CAD beats the robot's CAD for a library asset: use it if it is in the robot
  folder or the library's HP collections; otherwise build from the robot's and mention in the summary that vendor CAD
  would improve it.
- Placing: fit on frame-fixed parts only (trimmed point-to-surface ICP; whole-module ICP and plate-only ICP slide), then
  refit each moving sub-assembly (wheel fork) on its own CAD counterpart; check axle height and direction numerically.
- Any change to a module asset: rebuild, reinstall, rerun or refit every robot that uses it.

## 3. Compliant, flex and other wheels

- **`wheel_asset_cad.py`** builds a compliant wheel in the library's existing style from the CAD wheel's mid-width
  section: OD, width, hub radius, window count and shape measured; broken CAD windows repaired by circular pattern
  (STEP exports sometimes fan triangles across a window; never fill a window); windows mirror-symmetric with 12
  vertices, cut by one arrayed cutter; a solid hub cap only when the CAD has a separate hub insert; 16-sided tread;
  blue 50A / green 35A variants; HP / LP / Cutters / Asset collections. An optional suffix (", 16-sided") names an
  intentional variant. Fused hub inserts: split the insert out by material on a work copy first.
- **`wheel_asset.py`** builds from a spec (od, w, ring_in, hub, spokes, spoke_w, corner, bore, hex_on_spokes,
  material); measure the spec from the CAD (spoke count by angular FFT, ring/hub radii from radial histograms or ray
  parity of the solid fraction per radius at the mid-width plane, cast from the mid-plane, not from outside).
- **`flex_widen_asset.py`**: a straight flex wheel at another width = an existing width stretched along its axis.
- Pocketed wheels (windows from both faces with a mid-width web) fail the mid-section builders: use the stacked-section
  remodel (`remodel.py`) with a live Smooth by Angle + WeightedNormal.
- **Vectored / roller wheels** (`vectored_asset.py`): plain solid hub (only where a shaft fills the bore), rollers as
  convex rounded barrels clipped 0.25 mm inside the hub's end planes (even if the CAD rollers poke out).
- **Omni wheels exported as one fused body** (`split_extract.py` → module config → `omni_asset.py`): split by colour +
  islands; revolve each open roller shell about the rim tangent through a circle fit of its mid-length section (the
  lathe picked radial axes and made domes); fill only the hub's small socket loops.

## 4. Cameras and Limelights

- **`cam_asset.py`**: rebuild a CAD camera piece by piece (mesh islands; welded multi-colour pieces split by colour):
  tiny and enclosed pieces dropped; round → lathe, square body + round neck → box + lathe, flat → stepped plate, small →
  box, else plug + dissolve + a tight Collapse; colours to the closest finish. `CAM_EXTRA` adds case pieces from
  another object, `CAM_LENS` plugs the case opening and cuts the lens at the case face. Weld unwelded glTF pieces first
  (islands are single faces otherwise). Sample every triangle of an n-gon in visibility tests (a dissolved face around
  a boss has its median centre under the boss). Take the lens direction from the glass.
- **`cam_finish.py`** for a lens recessed in a printed hood (CAM_LENS would cut the whole lens; the 32-ray test drops
  recessed pieces): the lens as ONE revolve of the CAD front profile (highest point per 0.1 mm radius), the back closed
  by filling the flange's own rim loop plus an inner cap facing the cavity, prune again with a one-face margin.
- Boxy electronics cases: Collapse damages holes and recesses (corner holes into triangles, LED recesses into streaks).
  Build them procedurally like the Limelight assets: `limelight2_asset.py` (case with corners concentric with the
  holes, chamfer, LED recesses, lens ring), `limelight3g_asset.py` (reuses the Limelight 4's own case and corner
  cutter, openings as boolean cutters measured with bisect slices and ray grids). Real colours from product photos
  ("CAD yellow" is meaningless).

## 5. Motors, batteries, signal lights and small electronics

- **`clean_remodel.py`** (motor / battery modes): motors as one lathe of the body bands (tolerance 0.25 mm, no groove
  closing, so colour bands survive), the mounting face as its outline with holes filled, the shaft as a hex prism;
  screws, pins, bearings, connectors and ports left out (a connector pocket as a plain inset). Battery = loft of ONE
  outline with rounded edges; every ring with the same vertex count (separately resampled rings twisted). Wavy loft
  surfaces need a lower final dissolve angle (`DISSOLVE_DEG_PARTS` ~0.5°).
- A round signal light or similar small part: weld, axis by revolve score, `build.build_revolve(..., nseg=12,
  remodel=True)`, colours mapped to finishes, canonical frame (Z up, origin at the mount plane).
- A second model of an existing part gets its own asset at the existing one's fidelity.

## 6. Kit parts (stock channels, plates, blocks, servos)

Robots built from a vendor kit: remodel the big, triangle-heavy kit components once into the library (in a vendor
subfolder) and reuse them; screws stay per robot. Specs live in the robot config (`cfg.ASSETS`, `cfg.LIB_SUBDIR`,
`cfg.KIT_OPTS`); `kit_build.py` runs extract → vis → pipeline (`KIT_ASSET=1`: pieces simplified one by one) →
`kit_asset.py` → `asset_prune.py` → `kit_cmp.py` per asset, under one guard. Onshape assembly empties sit at the
origin, so assemblies are built in a key piece's frame and placed by each instance's key piece (`LIB_EMPTY_IDENTITY`).

`remodel.py` builders (through `cfg.BUILDERS`): `slab_union` (channels and flat sheets with every hole and slot),
`remodel` (stacked sections + cutters for side holes; `cyl=True` adds cylindrical wall patches; `snap=True` snaps
near-identical neighbouring outlines so no thin grooves appear), `reduce_holes` (perforated surfaces), `lathe_plus`
(lathe + sticking-out features such as clamp ears), `remodel_silhouette` (one axis intersected with the outline along a
second axis: rounded ear ends). Never Merge by Distance their output.

## 7. Motor caps and other accessories

An accessory for a library asset is built from the library asset's own geometry, never traced from one CAD part: the
motor cap (`kraken_cap_asset.py`) is the library motor's outline at its rear step floor, clipped at the notch wall and
extruded to the back face (20 tris), placed through the motor's library fit. One traced from a CAD cap sat 0.4°
off-angle, 0.2 mm into the wall, and left a ledge.
