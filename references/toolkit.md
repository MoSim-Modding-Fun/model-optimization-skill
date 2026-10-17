# The toolkit

`toolkit/` holds the headless Blender scripts (tested on Blender 5.x). Run them as
`<guard> blender -b <file> --python "$TK/<script>.py" -- <args>`; per-robot settings come from `robots/r<name>.py`,
selected with `ROBOT=<name>`. Robot configs live in your own folder (config `robot_config_dirs`); `toolkit/robots/`
holds only module configs and `rexample.py`, a commented template. Paths (the asset library) come from
`toolkit/paths.py` (config file or environment), never from the scripts.

## Contents
1. Running a robot
2. What `pipeline.py` does
3. Config reference (`robots/r<name>.py`)
4. Script catalogue

## 1. Running a robot

```bash
TK=<skill>/toolkit; S=<skill>/scripts; W="<robot>/_work"; SRC="<robot>/<source>.blend"
export ROBOT=<name>                                                    # robots/r<name>.py
# 0. prep: orig_names.json, tf_src.json, cad_mats.json, weld, explode cfg.EXPLODE -> $W/exploded.blend
"$S/bl.sh" "$SRC" --python "$TK/prep.py" -- "$W"
# 1. visibility (~1–5 min)
"$S/bl.sh" "$W/exploded.blend" --python "$TK/vis.py" -- "$W/vis.json" 80 1400
# 2. simplify (~10–45 min; planning, library fits and hole ray tests dominate)
"$S/bl.sh" "$W/exploded.blend" --python "$TK/pipeline.py" -- "$W/vis.json" "$W/run1/out.blend" "$W/run1/report.json"
# 3. exploded pieces back into their objects, final dissolve + shading + colour sync
"$S/bl.sh" "$W/run1/out.blend" --python "$TK/implode.py" -- "$W/run1/imploded.blend"
"$S/bl.sh" "$W/run1/imploded.blend" --python "$TK/phase3.py" -- "$W/run1/final.blend" "$W/run1/phase3.json" --cad-mats "$W/cad_mats.json"
# 4. QA renders
"$S/bl.sh" "$SRC" --python "$TK/render_pairs.py" -- before "$W/run1/pairs.json" "$W/run1/pairs"   # once per source
CULL=1 "$S/bl.sh" "$W/run1/final.blend" --python "$TK/render_pairs.py" -- after "$W/run1/pairs.json" "$W/run1/pairs"   # culled like Unity
LOCK=0 "$S/guard.sh" python3 "$TK/make_sheets.py" "$W/run1"
"$S/bl.sh" "$SRC" --python "$TK/compare.py" -- "$W/cmp/before" 0 clean
"$S/bl.sh" "$W/run1/final.blend" --python "$TK/compare.py" -- "$W/cmp/run1" 1
# 5. checks
"$S/bl.sh" "$W/run1/final.blend" --python "$TK/verify.py" -- "$W/orig_names.json" "$W/run1/verify.fbx"
"$S/bl.sh" "$W/run1/final.blend" --python "$TK/tfcheck.py" -- check "$W/tf_src.json" "$W/orig_names.json"
"$S/bl.sh" "$W/run1/final.blend" --python "$TK/report.py" -- "$W/run1/report.json" "$W/run1/report.csv" "$W/run1/totals.json"
"$S/bl.sh" "$W/run1/final.blend" --python "$TK/hexcheck.py" -- "$SRC" "$W/run1/hexcheck.json"
"$S/bl.sh" "$W/run1/final.blend" --python "$TK/capvis.py" -- "$SRC" '["<object>", ...]' "$W/run1/capvis.json"
"$S/bl.sh" "$SRC" --python "$TK/tubevis.py" -- "<object>" "$W/tv/<object>"
```

The before pass writes `pairs.json` (the part list and framing) and the `_before.png` images; the after pass adds the
result's tris to `pairs.json`. `make_sheets.py <dir>` wants `pairs.json`, `report.json` and `pairs/` in one folder:
render the before pass once, then copy `pairs.json` and the `_before.png` files into each later run's folder. Library checks: `libcheck.py dump` on the source with the swapped objects' names,
then `check` on the result with the run's `report.json`.

## 2. What `pipeline.py` does

Per unique mesh datablock (instancing makes this ~4× faster than per object), replacing mesh data in place, so object
names, parents, transforms and collections never change.

1. **Inventory** every mesh object (tris, modifiers, shared datablocks, world bboxes). With `PRE_WELD_M` it welds
   first; with `MERGE_DIST` it runs Merge by Distance before decimation and in the final pass.
2. **Visibility** (`vis.py`, run before): every mesh object gets a flat colour; ortho renders from 80 Fibonacci
   directions; pixels counted per object, on the whole robot and on each rigid group in isolation (`cfg.group_of`;
   `"main"` = the static chassis and the hole occluders). 0 px in its own group → `REVIEW - Hidden`; 0 px only in the
   full robot → `REVIEW - Hidden behind other mechanisms`; ≤ 4 px → `REVIEW - Barely visible`; exact duplicates (same
   polycount + bbox) → `REVIEW - Duplicates`, the kept copy inheriting the best pixel counts of its set (coincident
   copies z-fight in the id renders). Ids are 12 bits: at most 4,095 mesh objects.
3. **Method per mesh**, first match wins, each before the next for a reason:
   1. The owner's own modifier stack → keep. `KEEP_AS_IS`, or ≤ 30 tris → unchanged.
   2. `LIB_RULES` / `LIMELIGHT_PARTS` → **library swap** (PCA + ICP over 48 starts; accepted under 3 mm mean
      distance; among fits within 10 % of the best, the rotation closest to identity). Library before geometry: a
      motor's identity matters more than its tessellation.
   3. Box tube (`cfg.is_tube`) → **procedural tube**: outer box, wall thickness from the end cap (or from the inner side
      faces when the cap is incomplete), every CAD hole re-made as a hexagon through the wall; a hole is skipped when
      covered (rays 3 mm above it hit another part) or invisible (none of 32 hemisphere rays escape past static
      structure or the floor). Tubes rotated inside their own mesh are analysed in their own frame. Notched or mitred
      walls → wall slabs from each wall's outline (`TUBE_MITRED` for 45° ends). Before plate/lathe: tubes would
      otherwise be decimated and lose clean holes.
   4. Chain → **one closed loop** (`cfg.BUILDERS` → `chainloop.build_chain_loop` for a run fused in one mesh;
      `chain_join.py` for runs exported as separate links). The older per-link builder (`cfg.is_chain`) only makes
      REVIEW copies now.
   5. `FORCE_METHOD` overrides (mesh datablock name first, then part name): `keep`, `decimate`, `lathe`, `lathe-open`
      (open bore: cups, drums), `section lathe` (union of 24 axial cross-sections revolved: open cups with a floor and a
      boss; not spoked gears), `stacked lathe` (exact hex on hex lengths, lathe rings on round lengths), `sectioned`
      (each hex/round length its own closed prism, hex found by the 6th harmonic on 1 mm bins: often better than
      stacked lathe), `smooth lathe` (4 mm profile tolerance: nubbed treads), `hex` (no roundness test), `head cylinder`
      (rivet head only), `plate` (skips hex/lathe tests; can silently fall back to Collapse), `kraken cap`.
   6. Compliant wheels without a library asset (`WHEEL_SECTION`) → mid-width section extruded; eyebolts (`EYEBOLTS`) →
      shank + torus eye; coil springs (`SPRINGS`) → fitted helix (~2.8k tris; a plain cylinder is the 128-tri option);
      belts (`cfg.is_belt`) → toothless loop at the right thickness.
   7. Not round, but the outline fits a hexagon within 0.4 mm at the flats AND the corners → **6-sided prism** over the
      hex length (turned-down round ends become stubs). Hex before lathe: a hex would lathe into a rod.
   8. Round (roundness score, round-part keywords at word starts, `ROUND_EXTRA`, never anything named "plate") →
      **lathe**: radial profile from 60k samples, median-filtered, Douglas–Peucker, revolved at the side-count tiers;
      never accepted if heavier than the original; one majority material per ring; `LATHE_REMODEL` closes grooves and
      bands into plain cylinders and snaps to exact steps. Bores kept when visibly open, left open (not capped flush)
      where a shaft end sits in them.
   9. Plates (faces only parallel or perpendicular to one axis) → **stepped 2.5D rebuild**: outlines per height level,
      constrained Delaunay, explicit winding; small (≤ 3.5 mm), filled and invisible holes filled in; validated against
      the CAD (filling may only add flat area); open edges → rejected → decimate fallback.
   10. Long constant-profile parts → **end-profile extrusion**, accepted only if the p99.5 error < 1 mm (holes don't
       count in that error: a rotated tube once lost every hole this way).
   11. Everything else → **decimate**: plug small through-holes (< 8 mm), planar dissolve 5° (shape-exact), then
       Collapse to max(200, 8 × diagonal mm) tris, capped at 1,000; a quality loop raises the ratio × 1.5 until the p99
       error ≤ max(0.5 mm, 1.2 % of the diagonal), back faces ≤ the CAD's + max(1, 15 %), round holes ≤ 0.35 mm p95 at
       wall mid-points (at most 3× the start). `DECIMATE_RATIO` pins a ratio (it applies to the plugged + dissolved
       mesh, and is ignored for parts the "already light" check keeps).
4. **Multi-piece library swaps** (`LIB_ASSEMBLIES`, `LIB_COMBOS`), leftover CAD pieces inside swapped bodies → `REVIEW -
   Replaced by library`; motor caps where the CAD had one.
5. **Material transfer**: each new face takes the nearest original face's material (bearings: one area-majority colour
   voted by the CAD's face areas).
6. **Final shading pass**: Limited Dissolve ≤ 5° (delimit normal + material) on every mesh (`LATHE_DISSOLVE_DEG` ~1° for
   lathe/hex: shallow cones merge into non-planar n-gons at 5°; `DISSOLVE_DEG_PARTS` for gently curved sheets),
   concave n-gons re-triangulated; Smooth by Angle (30°) → WeightedNormal (Face Area, 50, Keep Sharp) as live
   modifiers on decimated and untouched CAD parts (faces set smooth first, or the stack does nothing) plus
   `SHADE_EXTRA` / `SHADE_LATHE`.
7. REVIEW collections, `report.json`, unused material slots dropped, linked data made local, save.

## 3. Config reference (`robots/r<name>.py`)

`robots/rexample.py` is the template. Everything is optional except where noted.

| Name | What |
|---|---|
| `ROOT` | Root object name (verify.py). |
| `part(name)` | Object name → part name (strip `.NNN`, `~n`, exporter suffixes). Required. |
| `group_of(obj)` | Rigid moving group for vis.py; `"main"` = static chassis. Required. |
| `SEE_THROUGH_GROUPS` | Groups that never hide anything and fill no hole (clear panels; objects a game prefab never draws). |
| `LIB_RULES` | `(part or mesh name, library .blend relative to Components/, asset object name)`. |
| `LIB_FLIP`, `LIB_MAX_MM`, `LIB_NUDGE_MM` | Per part: a turn about the asset's own axis (`"X"`, `"Z90"`, `"X+Z90"`…), a fit limit instead of 3 mm, a world-mm shift. |
| `LIB_ABSORB` | `{library part: {other part names}}` swallowed by the swapped body → REVIEW. |
| `LIB_ASSEMBLIES` | Multi-piece swaps: `select`, `groups`, `target: "largest"` or `anchor` + `object`, `lib`, `asset`, `label`, `flip`, `max_mm`, `bbox_tiebreak`. |
| `LIB_COMBOS` | `{part: [(gearbox lib, asset), (motor lib, asset)]}` for a gearbox + motor in one mesh. |
| `LIB_IDENTITY`, `LIB_EMPTY_IDENTITY`, `LIB_IDENTITY_MOTORS` | Assets built in the part's / an assembly empty's frame; library motors as separate children of module assets. |
| `LIB_CAD_COLOURS` | Parts whose library swap takes the CAD's colours (wheels, treads, modules: see the colour rule). |
| `LIB_KEEP_HEAVIER` | `"all"` (template default) or `True`: every library swap goes ahead even when the asset is heavier than the CAD part (owner: library by default, both shown at the end); or a set of part names / `LIB_ASSEMBLIES` labels. Unset, both paths keep a lighter CAD part. |
| `LIMELIGHT_PARTS` | Parts fitted against the Limelight assets; a dict forces one (`{part: "Limelight 3G"}`); takes no flip. |
| `KRAKEN_CAP`, `KRAKEN_ASSET` | Motor caps on/off (default on: only where the CAD had a cap). |
| `KEEP_AS_IS`, `REBUILD_KEEP_ALL_HOLES` | Part-name sets. |
| `is_tube`, `is_chain`, `is_belt` | Detection hooks. `TUBE_SIZES_MM` adds stock sizes (metric tubes). |
| `FORCE_METHOD`, `DECIMATE_RATIO`, `BUILDERS` | Per-part overrides (mesh datablock name first); `BUILDERS = {method: fn(part, mesh)}` for custom builders (pinned Collapse, plugged blocks, chain loops, remodels). |
| `EXPLODE` | Merged sub-assembly part names split into pieces before the pipeline. |
| `PRE_WELD_M`, `MERGE_DIST` | Weld (1e-6 for glTF soup); Merge by Distance (≤ ~5e-5). |
| `MESH_PREP` | `{mesh name: fn(me)}` edits before planning (flatten a logo cover, recolour bearings to their majority finish). |
| `LATHE_REMODEL`, `SHADE_LATHE`, `LATHE_MATERIAL_RINGS`, `LATHE_TOL_MM`, `LATHE_INSET`, `LATHE_DISSOLVE_DEG` | Lathe behaviour: remodel; shading stack on lathes; colour rings (a callable: off for bearings); per-part profile tolerance; pull end faces in where they sit flush; dissolve angle. |
| `HEX_BODY_FIT`, `FORCE_HEX_PLAIN`, `HEX_THROUGH_BORE` | Hex fitting on the shaft body; one plain prism without stubs; keep bores (leave OFF: hex stock is solid). |
| `ROUND_EXTRA`, `ROUND_BY_SHAPE`, `REV_KEYS_EXTRA`, `SPHERES` | Extra round parts; nameless round parts by shape; extra keywords (word starts; never "nut"); balls (must be as deep as wide). |
| `PLATE_TOL`, `PLATE_HOLE_SIDES` | Plate outline tolerance per mesh/part; side counts for small plate holes. |
| `DECIMATE_KEEP_HOLES`, `DECIMATE_MAX_TARGET`, `DECIMATE_LIGHT_FRAC`, `FLUSH_BORE_KEEP_STEPS` | Decimate fallbacks keep visible holes (`"all"` or a set); Collapse target cap; "already light" share; keep counterbore steps on reopened bores (default off: it inflated wheels 2–4×). |
| `TUBE_MITRED`, `TUBE_THROUGH_TEST`, `TUBE_FILL_IRREGULAR` | Mitred tubes as slabs; cut a wall loop only if rays cross the tube (engraved truss pockets stay solid); fill engraved script. |
| `NO_FLOOR_ALL`, `NO_FLOOR_OCCLUSION`, `FLOOR_Z`, `HEAD_CYL_NO_OCCLUDE` | No floor occluder at all (template default True: the underside is seen); per-part version; the floor height (helper cubes below the wheels) when a floor is used; rivet-head rebuilds don't occlude. |
| `VIS_DENSE` | Hole visibility from the centre + 6 rim points (slots seen only straight-on). |
| `EXTRA_DUPLICATES`, `EXTRA_HIDDEN`, `EXTRA_BARELY`, `REVIEW_AFTER` | Objects forced into REVIEW collections (coincident twins with different meshes; owner requests; parts the simplification itself hides). |
| `SHADE_EXTRA`, `SHADE_SKIP`, `DISSOLVE_DEG_PARTS` | Force/skip the shading stack; per-part final dissolve angle. `SHADE_SKIP` on smooth decimated parts makes them fully smooth (worse): lower the Smooth by Angle angle instead (20° when bend facets are 21–23°). |
| `EYEBOLTS`, `SPRINGS`, `WHEEL_SECTION`, `NORMALS_PARITY` | Special part classes. |
| `VIEWS` | compare.py cameras `{name: (target, direction, distance)}`. |
| `LIB_SUBDIR`, `ASSETS`, `KIT_OPTS` | Kit asset builds (`kit_build.py`). |

## 4. Script catalogue

| Group | Scripts |
|---|---|
| Core | `pipeline.py`, `vis.py`, `geo.py`, `build.py`, `align.py`, `shading.py`, `libcombo.py`, `paths.py` |
| Prep / big CADs | `prep.py`, `explode.py`, `implode.py`, `phase3.py`, `cleanup/` (Phase 1), `step_import.py`, `step_build.py`, `fbx_import.py`, `split_extract.py` |
| Special builders | `chainloop.py`, `chain_join.py`, `majlathe.py` (majority lathe: cups kept with their boss), `remodel.py`, `clean_remodel.py`, `patch_final.py` (edit a finished file: Collapse ratios, per-piece hex, plug + stepped) |
| Library and assets | `wheel_asset_cad.py`, `wheel_asset.py`, `flex_widen_asset.py`, `vectored_asset.py`, `omni_asset.py`, `cam_asset.py`, `cam_finish.py`, `limelight2_asset.py`, `limelight3g_asset.py`, `fan_asset.py`, `kraken_cap_asset.py`, `asset_prune.py`, `gapcheck.py`, `render_collection.py`, `swerve_*.py`, `mk4n_*.py`, `mk5n_*.py`, `x2i_*.py`, `kit_*.py` |
| Drop-ins | `game_restore.py`, `fbx_dropin.py`, `game_remake.py`, `fbx_normals_check.py`, `unity_probe/` |
| QA | `render_pairs.py`, `make_sheets.py`, `sheets.py`, `compare.py`, `verify.py`, `tfcheck.py`, `report.py`, `rundiff.py`, `libcheck.py`, `hexcheck.py`, `capvis.py`, `tubevis.py`, `flip_backfacing.py`, `pick.py`, `part_shots.py`, `context_shots.py`, `decision_shot.py`, `object_shots.py`, `object_compose.py`, `snap_state.py` (diff two saves), `finalize_mats.py` |

Each script's header comment gives its usage line; read it before running a script you haven't used.
