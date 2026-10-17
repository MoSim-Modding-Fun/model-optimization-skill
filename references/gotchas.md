# Gotchas: lessons learned the hard way

Search this file when something looks wrong; skim the bold leads once before a first robot. Each lesson cost at least
one rerun or review round. Tool behaviour is described as of the current toolkit.

## Contents
1. Names, detection and identity
2. Visibility and holes
3. Lathes and round parts
4. Hex stock
5. Box tubes
6. Plates
7. Decimation (Collapse)
8. Shading, normals and z-fighting
9. Chains, belts, springs, wheels
10. Materials and colours
11. Library fits
12. Files, matrices and Blender API

## 1. Names, detection and identity

- **Keywords match at word starts only.** "ring" in "gas spring" sent an eyelet to the lathe; "shaft" in "Shaft
  Retention Plate" made a disc; "belt" in "Bottom Belt Gearbox Plate" made a belt; a "nut" keyword lathed a square nut
  strip into a rod (nuts pass the hex test anyway, don't add "nut"). Nothing named "plate" is ever lathed: a sprocket
  named "… Plate Sprocket" was decimated with all its teeth; force `"lathe"` for such parts.
- **Names lie.** A part called "…Chain" was a clamp block; "Wheel, …, Bearing Bore" is a hub; a "Kraken … Back Cap" was a
  printed cover. Check a render before trusting a name for method detection.
- **Generic names cover unrelated parts** ("Part 40" = a motor in one module and a spacer in another; "Part 1" = a
  pulley and a bumper). Key `FORCE_METHOD`, `DECIMATE_RATIO`, `LIB_NUDGE_MM` etc. on the mesh datablock name.
- **A sister config's choices don't transfer.** A camera model forced in one robot's config was wrong for the next.
- **Coincident duplicates with different meshes** (a second design iteration of a belly pan 98 % within 0.05 mm of the
  first, a pulley modelled twice, a "-Mirrored" twin on its original) aren't caught by the exact-duplicate test and
  z-fight after the rebuild makes them coplanar. Sample one, measure the nearest distance to the other; the less visible
  one goes in `EXTRA_DUPLICATES`.
- **Count every motor** in the CAD and confirm each got a library swap (multi-piece motors are easy to miss).
- **Bodies can be missing from an export**: compare a part's size with its name.
- `vis.py` encodes object ids in 12 bits: more than 4,095 mesh objects alias silently.
- **Belts named only by part number** (custom HTD belts, turntable belts with a pinion tail) skip `is_belt` and get the
  plate rebuild with every tooth (1.3–4.4k tris each). After run 1 list the dark loops 9 / 15 mm thick (or whatever the
  belt widths are), look at them, and add their part names to `is_belt`.
- **Anchors by local `dimensions`** miss pieces that sit at an angle in their own mesh (a 75.6 mm motor body read 66 mm:
  a `> 70 mm` anchor test found none, and the multi-piece swap silently never ran). Anchor by face count or PCA extent,
  and check that every `LIB_ASSEMBLIES` label appears in the pipeline log.

## 2. Visibility and holes

- **Two-level visibility**: each rigid group in isolation (so a part hidden only by a stowed mechanism isn't "hidden")
  plus the whole robot. Small parts at ~1 mm/px can be missed: barely-visible parts go to REVIEW, not away.
- **Holes behind moving groups stay**: occluders are the `"main"` group plus the part's own group. Put fixed roller sets
  in "main" only if they truly don't move.
- **Clear panels** must be a see-through group, or parts behind them go to REVIEW and visible holes get dropped.
- **The underside is seen** (owner default, `NO_FLOOR_ALL = True`): no floor occluder in hole tests. Holes on rails'
  undersides were dropped on earlier robots and later wanted. If a floor is used, helper cubes below the wheels drop
  it: set `FLOOR_Z`.
- **Hole plugging is the main source of see-through spots** on decimated parts:
  - pins and bosses have two round rims like a hole; plugging one deleted its wall (only plug when the wall faces its
    axis);
  - caps get no orientation from the normals pass on unwelded CAD (orient each cap like the faces around its rim);
  - plug caps take the colour of the surface around their rim (a white gland end turned red when it took slot 0);
  - size holes by cross-section (middle bbox extent) and rim diameter, not the longest side (a 4–5 mm hole through a
    12.7 mm boss measured 12.7 and wasn't plugged; Collapse then made 25 diamonds);
  - a plug cap on a flush boss sits exactly on the other part's face: leave the hole open when a same-facing face fills
    it flush (opposite-facing contact is fine);
  - blind holes (dimples) can't be plugged: Collapse then crumples the block; a solid block inside a tube is better as
    one 12-tri box;
  - iterate bmesh elements in index order, never through a `set` (address order): the same plate plugged 0 or 25 holes
    in different processes.
- **"Filled by a bolt" holes can still be seen** (a 17 mm hole seen by 11 % of rays; tube-plug end holes by 25 %).
  capvis decides.
- **Coplanar layers**: opening one layer's holes exposes plugs in the next (belly pan → plate → supports). Fix the whole
  stack; a decimated part and a plate rebuild can disagree on the same holes.
- **Countersunk and king-pin holes** fool the "bolt in it" ray test (rays start at 0.4 × the wide countersink rim):
  `DECIMATE_KEEP_HOLES` per part.
- **Rows of open 4–5 mm holes on big decimated sheets** (hopper walls, an intake hood) were plugged and nobody saw it on
  the per-part sheets: at 256 px the holes are a pixel. The whole-robot views showed the dot rows missing. Compare the
  robot views for hole rows on every large sheet-metal part, then `DECIMATE_KEEP_HOLES` per part (`"all"`).
- **Lathes drop holes through the side of a round part**: a 565 mm thin band lost 32 open 7 mm T-notches, and a
  pocketed ring two open 19 mm holes. Band: `"decimate"` with its holes kept and a pinned ratio; 0.15 (1.6k tris) turned
  the T-notches into funnels with shading streaks, 0.30 (3.1k) kept them. Ring: `"decimate"` with `"all"` holes at
  0.25 (2.1k; 0.40 looked the same at 3.4k). Look at notches and pockets in a close-up before pinning a ratio.
- **`remodel.slab_union` is for walls of one thickness**: on a ring with 5 mm pockets in a 6.35 mm plate it kept only the
  1.3 mm web (rim, ribs and bosses gone, the countersunk screws floating 5 mm above it). Its `err_mm` (p95 CAD→slab,
  slab→CAD) said so, and so did every other value above ~0.5 mm: 5.07 / 1.27 there; 3.17 / 3.17 on a plate whose 6.3
  mm ring and spokes in a 3.1 mm frame came out all 3.2 mm (a later review round found it); 1.27 / 1.27 on a floor
  whose underside recess was filled. Exact slabs read 0.01 (mitred tubes). `analyse_stepped` returning None (not a
  stepped plate) is the cue for Collapse instead: 0.25 with every hole kept both thicknesses and round small holes.
- **`"sectioned"` is for shafts, not a gear on a stub**: its axis came from PCA, which put it in the gear's plane (a
  tilted stack of 15 slabs, bigger than the part; fixed: the best revolve-score axis now), and on the right axis it
  read the gear as a ring with a wide bore (844 tris vs the CAD's 804). A part under ~1k tris whose rebuild is no
  lighter stays as the CAD (`"keep"`).
- **Flat sheets whose plate rebuild is rejected for open edges**: Collapse at its automatic target turned their 4–5 mm
  holes into diamonds, and `DECIMATE_KEEP_HOLES` `"all"` left a 571 mm disc at its CAD count (17.3k).
  `remodel.slab_union` gave 2.0–2.3k tris with every hole as a hexagon, but at one thickness only (the `slab_union`
  item above). A pinned `DECIMATE_RATIO` of 0.25 with `"all"` holes kept a two-thickness plate's holes round at 1.9k.
  Check the report's tris after any keep-holes change.

## 3. Lathes and round parts

- **Remodel, don't sample**: the sampled profile turned evenly spaced belt grooves into a saw-tooth that looked
  decimated (`LATHE_REMODEL`). Without the 1/3-length cap, closing flattened screw domes and rivet heads into pucks.
- **The remodel can flatten crowned or barrel profiles** (mecanum rollers) into cylinders: test one part first.
- **Open C-shaped bands score as perfectly round** (0.0003): the lathe closes the gap into a ring → `"decimate"`. Check
  angular coverage, not just roundness. A FLAT C-shaped spacer ring with bolt holes was lathed into a washer: `"plate"`
  keeps the gap and the holes.
- **Cups**: every per-height profile closes a recessed cup. `"lathe-open"` keeps a bore; `"section lathe"` keeps cups
  with a floor and a boss (not spoked gears: 24 cuts miss narrow spokes and leave a floating hub); the majority lathe
  (`majlathe.py`: a cell is solid where > 50 % of the circumference is material) keeps a recessed cup with its boss and
  drops ribs; give it only to instances whose cup faces open space.
- **Two rings in one mesh** (tapered roller cup + cone) get bridged into one tube: lathe each island on its own axis.
- **Bores**: "largest sector minimum" over sparse samples jumped to the next surface (a spoked gear became a bare ring);
  sectors above 1.35× the median are ignored now. A bore capped flush with a shaft end z-fights: left open; the test is
  re-run after library swaps (a library motor's shaft can make a bore flush). The post-swap reopen has a weight check
  (a 216-tri shaft once came back as 10,656 tris). `FLUSH_BORE_KEEP_STEPS` keeps counterbore steps (2–4× heavier wheels:
  default off).
- **Slender parts** (bolts, threaded rods): the 90th percentile of the sector maxima (major diameter); the median made
  bolts ~25 % too thin.
- **Disc-like parts with recessed faces** (gear webs): a radial profile (face height per radius) instead.
- **Flush ends**: an end face lying on another part is pulled in 0.15 mm; partial contact (inserts flush inside a
  sprocket's windows) needs `LATHE_INSET`.
- **Lathe colours**: one majority material per ring (per-face nearest material made radial stripes on spoked gears).
  Bearings: one colour voted by the CAD part's face areas (voting on the lathe's faces left a shield-coloured ring).
- **Spheres**: every rim point of a flat disc lies on a sphere; balls must be about as deep as wide (`SPHERES` turned decal
  discs into balls).
- **Smooth lathe** sits ~0.13 mm inside a nubbed tread's tips (matters if anyone measures ground contact), and on a
  flanged pulley it made a cone.

## 4. Hex stock

- **Checking only the six flats lets a round part pass** (a circle touches all six); the outline must also reach the
  corners (a 14-tooth pinion became a hex prism).
- **Rounded-hex stock and shafts with clips or turned ends fail the automatic hex test** and get lathed into rods; the
  hull of all vertices takes in end clips (1/2" hex came out 14.4 mm across flats): fit the hexagon to body samples
  (`HEX_BODY_FIT`) and force `"hex"` for parts with a strong 6th harmonic.
- **`"stacked lathe"` can swap hex and round lengths** and miss a hex top; `"sectioned"` (6th harmonic on 1 mm bins) was
  right where stacked was off by 4.4 mm. Test both on the part. A part that is a round tube with a hex middle needs
  `"sectioned"`, not `"hex"`.
- **Nylon-insert lock nuts** got a fake journal stub: `"hex"` + `FORCE_HEX_PLAIN`; screw tips then sit flush on the
  solid nut (`LATHE_INSET`).
- **Closed prisms meeting end to end** z-fight at the junction: internal junctions overlap by 0.1 mm.
- 12-tooth splines carry a 6-fold pattern too: a length counts as hex only if its outline also fits a hexagon within 6 %.

## 5. Box tubes

- **Metric tubes** (25 × 50 mm) miss the imperial size test by 0.8 mm and fall to the profile extrusion, which drops every
  hole: add the sizes to `TUBE_SIZES_MM`. Short tubes (a 2×2 tube 96 mm long; a 7" 2×1) fail a "length > 4× section"
  rule: 3× works; visible 2×1 tubes 4" and 4.5" long fail 3× too and were decimated without a hole (1.8× works).
- **Round tubes whose OD is a stock box size** (1.5" and 2" dead-axle tubes) pass the bounding-box tube test and came out
  as three slivers: reject round sections in `is_tube` with `geo.revolve_score` along the tube axis (box tubes score
  0.13–0.29, round tubes ~0.002; a > 0.03 cut separates them).
- **A tube rotated inside its mesh** (4° or 35° about its own axis) fits no size and became a profile extrusion or a
  bbox-sized box without holes: detect the frame from the wall normals first, whenever the section is roughly
  tube-sized.
- **C-shaped end caps** (one wall's end face missing): wall thickness from the inner side faces; gate it on a square cap
  existing (mitred tubes are a different case: `TUBE_MITRED`, wall slabs from the outlines, holes kept).
- **Notched walls** stay open: a plain tube closed a 51 mm end notch and hid the block in it.
- **Merged round-hole loops** (2–3 neighbouring holes traced as one loop through a CAD sliver) leave open edges: split
  back into circles (each piece a circle arc spanning > 270°, so real obround slots stay); a tube with open edges is
  rebuilt once without those holes, then T-junctions fixed.
- **Engraved truss pockets** have their outlines on the outer face like holes: 7,200 mm² of see-through windows were cut
  until `TUBE_THROUGH_TEST` (rays must cross the tube). Measure pocket depth before calling a pattern engraved: 2.4–6.5 mm
  pockets are visible features.
- A bent C-channel passes the tube bbox test: exclude it.
- **Mitred box-tube ends** (no `TUBE_MITRED`): the tube rebuild ran the side walls to full height while the top wall
  stopped short (an open U-channel end), and the mirrored copy failed the size test (the mitre widens its bbox) and was
  decimated to a plain box without its holes. `remodel.slab_union` through `cfg.BUILDERS` rebuilt all four as wall
  slabs with the mitre and every hole (0.01 mm, 250–530 tris). **`FORCE_METHOD` does not beat `is_tube`** (the tube
  test runs first): return False from `is_tube` for those meshes too.

## 6. Plates

- **Loop tracing** walks boundary edges, not vertices, and doesn't trust face winding (pinch vertices, mixed winding).
- **Circle detection is strict**: deviation < 1 % r AND area within 15 % of πr² (pocket cuts running off an edge and thin
  slots are "almost circles").
- **Hole pairing** (top ↔ bottom) pairs only real holes, never an outline, or half a plate disappears.
- **Levels simplified separately** left see-through slits in most stepped rebuilds (canonical loop start, a vertex kept
  on one level kept on all, T-junction split, slit fill). **The same outline tessellated differently on two levels**
  shared no vertices and the boundary fill spanned tooth gaps with flaps: reuse the first level's loop. Per-level circle
  fits differ by µm: one fit per physical hole.
- **Rejected for open edges**: about 40 % rescued by filling planar boundary loops; the rest fall back to Collapse, which
  turns kept 4–5 mm holes into triangles unless the fallback keeps them round.
- **Holes across the extrusion axis** disappear without failing the fit check: such plates go to the decimate fallback.
- **Countersunk holes** break the stepped rebuild (back-face check): plug them where invisible, keep the CAD where seen.
- A forced `"plate"` silently falls back to Collapse when faces aren't axis-parallel: check the report's method.
- Thin plates render nearly edge-on in iso sheets: add face-on renders along the plate normal.

## 7. Decimation (Collapse)

- **Plug, then planar dissolve, then Collapse**: plugging first stops Collapse tearing sheet metal full of holes into
  zig-zags; planar dissolve is shape-exact (a bumper frame went 15k → 1.2k unchanged).
- **Low ratios lie**: p99 error stays small while bosses and holes vanish (a bearing block lost its bosses at 0.2 with
  0.2 mm error). Use the error loop only to raise ratios. Watch p99.9 on prominent flat parts (a bar creased at 0.2,
  clean at 0.5).
- **Collapse is not repeatable** run to run (a slot tore through a thin tab in one run and not in a test): pin
  borderline parts with `DECIMATE_RATIO`, and patch decisions into the reviewed file instead of rerunning.
- **`DECIMATE_RATIO` applies to the plugged + dissolved mesh**, not the CAD (0.2 gave far fewer tris than expected), and
  is ignored for parts the "already light" check keeps: pin a Collapse builder for those.
- **The decimate prep's planar dissolve can cut a window**: on printed camera mounts the 5° dissolve left an n-gon that
  the concave-n-gon check missed and that was triangulated straight across the camera window; the quality loop then gave
  up (p99 error 15 mm, no Collapse) and kept 6–7.7k tris. Keeping holes did not help (a camera right behind a window also
  makes the plug test call it filled). A plain Collapse of the welded CAD at a pinned ratio, without the plug/dissolve
  prep (a `cfg.BUILDERS` builder + `SHADE_EXTRA`), keeps the window: 12k → ~1k at 0.08. Test ratios with culled renders
  from 3 sides first.
- **Normals before decimating**: a plain recalc can turn open CAD inside-out; per-piece orientation that agrees with the
  original normals, a parity vote only when it lowers back-face hits (a vote once flipped a whole housing to 97 % back
  faces). Never accept a normals pass that makes orientation worse.
- **Decimation prep can flip faces only culling shows** (fuse levers striped in Unity): culled QA renders, pinned plain
  Collapse for such parts.
- **Threads** keep failing the error loop on crests alone (eyebolts at 2k): shank + torus eye instead.
- **Engraved lettering**: plain Collapse erases flat engraved outlines at any ratio (their cost is ~0) and plugging
  erases small letters. Keep the part out of the pipeline and Collapse it with a vertex group: weight 0 on the letter
  material's faces plus 2 rings, weight 1 elsewhere, `vertex_group_factor` 1, from the welded CAD mesh. (A vertex group
  that marks holes does NOT protect them: the plain group stopped decimation, the inverted one flipped faces.)
- **Big curved parts**: choose the ratio from rendered error (`part_shots.py` ratio sweep + `magick compare -metric
  MAE`, 3 views, take the knee), not the lowest ratio that holds the shape. Bands of 400–800 mm needed 0.12–0.15, not
  ~0.05. Constant-section rings: a true revolve with analytic normals baked as custom normals (no shading modifiers on
  them: WeightedNormal would overwrite them).
- **Fused vendor housings**: ratios are relative to the export's density; match tri counts across exports.
- Retune live Collapse ratios in the finished file (`patch_final.py`) and check each in renders (0.06 closed a slit,
  0.025 made a dome lumpy).

## 8. Shading, normals and z-fighting

- **Limited Dissolve leaves long n-gons** that shade with streaks: Smooth by Angle 30° → WeightedNormal (Face Area, 50,
  Keep Sharp) after it. **The stack does nothing on flat-shaded faces**: set faces smooth first.
- **5° dissolve on shallow cones and chamfers** makes non-planar n-gons with patches: ~1° for lathe and hex parts. On
  gently curved sheets the merged strips cut into a neighbouring sheet: per-part 1°.
- **Concave n-gons** get re-triangulated (one wrapped around a wheel's windows was tessellated across them).
- **SBA 30° merges real bend facets of 21–23°**: lower the angle on that part rather than `SHADE_SKIP`.
- **8-sided parts stay faceted under SBA 30°** (45° corners): intended by the side-count rule; reviewers flag it.
- **Z-fighting comes from flush contacts the rebuild makes exact** (bores flush with shaft ends, bearing faces on
  plates, inserts in windows): default to a 0.15–0.3 mm inset or sink. Check claims by measuring both files.
- **An inset can land exactly on the neighbour's simplified face**: a bearing pulled in 0.15 mm met a plate rebuild
  whose top had dropped the CAD's 0.15 mm boss level, and heat-set inserts met decimated plates the same way; still
  coplanar after the fix. `LATHE_INSET` also applies to every lathe of the part name: on a 0.68 mm washer a 0.3 mm inset
  left 0.19 mm, and its face then met the nut's stub. Rerun `zfight.py` after every inset change, and fix the finished
  piece file with `zf_nudge.py` (moves the smaller part's coplanar face 0.15 mm in, deterministic) rather than a rerun
  that changes other Collapse results. One pass took 55 new pairs to 5 slivers.
- **Library meets library by fit coincidence**: 3 of 8 motors on library swerve modules had their top flush with a
  module face (the CAD had no overlap); the tops tilt ~0.15° from the fit, so a 0.15 mm move left part of one disc
  coplanar: 0.3 mm. Check the motor's other end: moving one up put its shaft end flush with a pinion.
- **`zfight.py` pairs of 1–3 samples change from run to run** (random samples): confirm with `zf_nudge.py`'s hit count
  or a 3000–6000 sample probe; edge slivers of a few mm² between stacked plates are not worth a fix.
- **Unity culls back faces; Blender's renders don't**: QA "after" renders with culling.
- **Unity drops normals** unless the model's Normals import setting is "Import".

## 9. Chains, belts, springs, wheels

- **Chain = one loop**: a closed band along the run's centreline (averaged over a pitch), offset ± half the plate height,
  extruded over the width (a 92-link run: 36,960 → 344 tris). Runs exported as separate links: join each run's CAD links
  once, build the loop, move the links to REVIEW. Chain links must not occlude: a chain through a slot closed the slot.
- **Belts**: inner loop = outer loop offset by the back + land thickness measured to the outer polyline *segments* (long
  straight spans have no vertices: belts came out 1.5–1.8× too thick); faces taken regardless of facing (mixed
  winding); raised lettering on a belt's face gives extra loops (fall back to the face with exactly two). Toothed belts
  that aren't named belts get the plate rebuild with their teeth: sweep shapes, not only names.
- **Springs**: fit the helix pitch and phase on the wire's outermost points (all points smear the phase), coil body from
  slices with points all round the axis, end loops by PCA plane + circle; hooks must not read as coil.
- **Compliant wheels**: broken CAD tessellation fans triangles across a window; repair by circular pattern, never fill a
  window. Look at CAD wheels with culling OFF before blaming the rebuild.
- **Solid plain hubs** (no bore) are fine only where a shaft fills the bore: check by the shaft line, not axial rays.

## 10. Materials and colours

- **Planar dissolve stops at material borders** (`delimit` material), or bumper lettering smears into one colour.
- **Linked library materials** leak into outputs: make everything local before saving.
- **Empty material slots** export as empty FBX materials: drop them (non-drop-in). A drop-in keeps them (filler
  triangle per slot).
- **`.001` duplicates** appear when assets bring their own copies of shared finishes: merge them when the colours match.
- **Unity URP reads only the FBX diffuse colour** (no metallic): a white metallic finish shows white.
- **Workbench renders metallic finishes near-black**: set metallic 0 for QA renders; compare face areas per material
  before reporting a colour bug.
- **Viewport colours** of materials appended after the pipeline stay at 0.8: sync from the Principled BSDF at assembly.

## 11. Library fits

- **Symmetric and near-square boxes fit 180° / 90° off** with the same error: check by a feature (asset-library.md §5).
- **Motors sit a few mm off along their axis** (the rigid fit centres the asset; a CAD motor longer at the back leaves the
  library motor short of its mount: a dark slit). Measure the front face along the axis.
- **Multi-piece motors** leave CAD connector slivers poking through the library body unless the pieces go to REVIEW.
- **Loose pieces with no assembly node** (33 pieces of one motor under an arm): group by touching bboxes; a "body =
  diagonal > 50 mm" rule split each motor into 6 groups and every fit failed.
- **A library asset can include a stage the CAD lacks** (a gearbox output stage): slide it onto the CAD's input face.
- **Library swaps on objects carrying a live Collapse** get decimated by it (a 12-tri "disc" in decision renders): strip
  Decimate modifiers when swapping.
- **Box electronics can fit a few degrees off their axes** (a PDH 85.9° instead of 90°: faces 4–8 mm off in libcheck):
  snap the fitted rotation (report.json `lib_matrix`) to the nearest axis-aligned one about the asset's own centre, then
  centre it by the libcheck face offsets, 0.3 mm short of the face it sits on.
- **Library meshes are shared**: a shift for some instances only (motors whose front boss lies in a module plate) goes on
  a mesh copy for those objects; object transforms must stay equal to the CAD (tfcheck).
- **A fit just over the limit on every copy** (Vortex + SPARK Flex: 3.65–3.74 mm on all 15 motors of one CAD, limit
  3.5) is a systematic difference between the CAD and the asset, not 15 bad poses: raise `max_mm` for that label and
  check orientation by renders; the pieces were otherwise kept as heavy CAD.
- **The library Limelight 3's front has no green LEDs** (a lens and two LED holes in black), so in a render it reads like
  a camera's back: a session "fixed" two correctly placed cameras with a flip and had to undo it. Judge orientation by
  the lens and LED-hole side against the CAD's lens side, from the same camera in both files. To undo a flip, take the
  meshes from a run without it (same fit, same object transforms); a 180° turn about PCA axes left them tilted.
- **`LIB_CAD_COLOURS` only applies to `LIB_RULES` swaps**, not `LIB_ASSEMBLIES`. A wheel with a separate hub insert:
  a `LIB_RULES` entry on the part name (the size gate, 0.6–1.6 × the asset's diagonal, keeps the small insert out) and
  the insert goes to REVIEW through the swapped body's box.
- **libcheck compares the asset with the largest CAD piece only**: a multi-piece swap shows the parts the target piece
  lacks as one-sided face offsets (a docked controller +13.4 mm, an encoder's back cover +4.8 mm). Read those against
  the assembly before flipping anything; a moved colour centroid is the stronger hint.

## 12. Files, matrices and Blender API

- **Record world matrices before moving pieces**; objects in excluded collections and appended-but-unlinked objects
  report stale or identity matrices until linked and evaluated.
- **Inventory world bboxes of rotated objects** read up to √2 too big: compare in local frames.
- **Move pieces into an excluded REVIEW collection only after baking live modifiers** (REVIEW counts doubled and an FBX
  grew from 7.6 to 13.9 MB) and assert the bake actually shrank the mesh.
- **Loaded meshes get `.001` suffixes** next to existing ones: map by request order, not name.
- **Geometry-node modifier inputs** (Blender 5.x): `mod.properties.inputs.<id>.value`. `system_resource('DATAFILES',
  path=…)` returns `''` for files: join the path yourself. `set_sharpness_by_angle` became `set_sharp_from_angle`.
- **`--factory-startup`** leaves the default cube, camera and light: `read_factory_settings(use_empty=True)`.
- **Names over 63 bytes** are cut by the FBX importer (use `fbx_import.py`); STEP builds shorten long names on purpose.
- **Object names containing "/"** become sub-folders in render output paths.
- **Blender exits 0 when a `--python` script raises**: a chained run script carried on with a stale file after a post step
  crashed. Run robot post steps with `--python-exit-code 1`.
- **`Mesh.materials.clear()` resets every face's material index to 0**: store the indices, clear, write them back with
  `polygons.foreach_set("material_index", …)`. Objects have `children_recursive` but no `parent_recursive`.
- **ImageMagick `montage` decodes every input at full size**: 60 per-object JPGs (up to 3200 px) passed a 2 GB job cap
  and were OOM-killed. Shrink each image on its own first (`magick -define jpeg:size=600x340 X -thumbnail 300x170`),
  then montage the thumbnails.
- **`MemoryMax` counts touched pages**: test the guard with memory that is actually written.
- **Pass absolute paths to render scripts** (`vis.py`): with a relative output path the first render failed under the
  guard ("cannot save vis.json.tmp.png") and Blender still exited 0.
- **Boolean modifier `material_mode` defaults to INDEX**: the operand's faces take the target's slot of the same index,
  so unions in an asset build all came out in the case colour. Use `'TRANSFER'`. Call `view_layer.update()` (and
  `depsgraph.update()`) before evaluating a Boolean on an object created in the same script.
- **Blender 5 keeps names over 63 bytes** (77-byte glTF node names arrived whole); older notes about 63-byte cuts apply
  to the FBX importer.
