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
  strip into a rod (nuts pass the hex test anyway, don't add "nut"). Nothing named "plate" is ever lathed.
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

## 2. Visibility and holes

- **Two-level visibility**: each rigid group in isolation (so a part hidden only by a stowed mechanism isn't "hidden")
  plus the whole robot. Small parts at ~1 mm/px can be missed: barely-visible parts go to REVIEW, not away.
- **Holes behind moving groups stay**: occluders are the `"main"` group plus the part's own group. Put fixed roller sets
  in "main" only if they truly don't move.
- **Clear panels** must be a see-through group, or parts behind them go to REVIEW and visible holes get dropped.
- **The floor occludes** hole tests by default; climbing or tipping robots may want `NO_FLOOR_OCCLUSION`. Helper cubes
  below the wheels drop the floor: set `FLOOR_Z`.
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

## 3. Lathes and round parts

- **Remodel, don't sample**: the sampled profile turned evenly spaced belt grooves into a saw-tooth that looked
  decimated (`LATHE_REMODEL`). Without the 1/3-length cap, closing flattened screw domes and rivet heads into pucks.
- **The remodel can flatten crowned or barrel profiles** (mecanum rollers) into cylinders: test one part first.
- **Open C-shaped bands score as perfectly round** (0.0003): the lathe closes the gap into a ring → `"decimate"`. Check
  angular coverage, not just roundness.
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
  rule: 3× works.
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
- **`MemoryMax` counts touched pages**: test the guard with memory that is actually written.
