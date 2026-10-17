# QA: proving a simplified robot is right

QA is not "send it to a reviewer". Past robots needed 2–14 review rounds, and the problems that mattered most were
found by the session's own look at every pair and by measurement scripts, or not at all by reviewers (broken Unity mesh
references, an emptied mesh after a restore, a motor 3 mm off its mount, a shaft that came out round). Run the
automated checks yourself after every run; give reviewers only what needs eyes.

## Contents
1. The loop
2. Automated checks after every run
3. Reviewer brief template
4. Verifying claims (false positives seen before)
5. Re-runs and late changes
6. Per-object images

## 1. The loop

1. **Renders.** `render_pairs.py before` (once per source; it strips the CAD's custom normals) and `CULL=1 … after`, then
   `make_sheets.py` / `sheets.py`. Render the AFTER pass with **backface culling** (Unity culls; unculled renders hide
   flipped faces and slits; `CULL=both` for single-sided sources). Workbench, cavity **off** (cavity makes thin rebuilt
   triangles look crumpled; the owner once called a clean plate "too low poly" because of it), metallic 0 for the
   render. Sheet labels must take the REVIEW status from the final file's collections, not the pipeline report (parts
   moved to REVIEW in post steps showed as visible). Exploded parents render as meaningless crops: filter or label them.
2. **Your automated checks** (section 2). Fix what they find before anyone reviews.
3. **Your own pass over every sheet, every pair**: blank tiles, `0>4`-style tri counts, the wrong method for the part,
   gears decimated with jagged teeth, hex parts lathed round, tubes with no holes, colour stripes on library parts,
   torn thin plates. Full-size images when a tile is too small; downscaled sheets alias into "speckle" that looks like
   z-fighting.
4. **Fresh reviewers**: `general-purpose` subagents with no session context, in the background, 2–3 per round, each
   with a slice of the sheets (one reviewer on 28 sheets loses detail; one took 89 min and 749 tool calls until asked to
   "look directly, diff only to confirm"). Brief: section 3. Images and read-only measurements only: no saving files,
   one Blender at a time, write only under `_work/reviewN/`.
5. **Verify every claim** (section 4) before fixing: about a third of late findings were false positives. Add each
   settled claim to the brief's "already measured" list, or the next round reports it again.
6. **Fix** with try-scripts on the 1–5 affected parts first (load the file, call the `build.*` function, render it next to
   the CAD) instead of a 15–45 min full run; patch small changes into the finished file (`patch_final.py`: live Collapse
   ratios, per-piece hex, plug + stepped rebuilds) rather than rerunning: Collapse is not repeatable run to run.
7. Repeat until a round comes back clean (no HIGH, no real MEDIUM). Budget 5+ rounds on a 3M-tri robot: reviewers dig
   deeper each round, and opening one layer of holes exposes the next.

## 2. Automated checks after every run (mandatory)

| Check | How | Why |
|---|---|---|
| Hierarchy and transforms | `verify.py` (names, parents, collections, FBX export) + `tfcheck.py` (every source object's world matrix vs the source: max deviation 0.0) | names and transforms are the contract with Unity and the owner's later edits |
| Totals | `report.py` from the FINAL file, tris with modifiers evaluated: visible / per REVIEW collection / by method | the pipeline's report is stale after implode, placement and patches; mesh-data counts overcount decimated parts |
| Open edges | 0 on every rebuilt mesh (lathe, plate, tube, hex, chain, belt, spring, eyebolt) | culled renders show slits as see-through cracks |
| Back faces | `build.backface_pct` per decimated or library mesh ≤ the CAD's own + max(1 point, 15 % of the CAD's) | never in absolute terms: some CAD parts are inside-out themselves |
| Hex stock | `hexcheck.py` (6th harmonic of the cross-section at ¼, ½, ¾ length: 1/2" hex ~0.43 mm, 3/8" ~0.32, a lathed 16-gon ~0.001), plus a list of every part with the robot's hex-stock cross-section looked at one by one | a 16-gon reads as hex at sheet size; four review rounds missed six round shafts. It skips short pieces (< 2× diameter: spacers, collars, nuts) and gives false "ROUND" on grooved hex stock (read the value) |
| Hole visibility | `capvis.py` on every "holes closed" claim before fixing (no `CAPVIS_FLOOR`: the underside counts as seen; `CAPVIS_CLEAR` for clear panels): vis = share of 96 hemisphere rays escaping; 0 never seen, < 0.02 glimpsed, > 0.1 clearly visible | 7 of 10 flagged parts once had vis 0. It flags any deviation > 0.8 mm (not only holes) and is an upper bound (it looks into cavities too) |
| Coplanar z-fighting | point-sampled: sample one part's faces, ray-cast ±0.02 mm onto same-facing faces of the other (within 0.05 mm and 1.8°), measure the gap in BOTH files, keep only pairs visible from outside | plane-offset comparisons (`n·p`) gave three false claims of 1.5–3.3 mm real gaps; pre-existing CAD z-fights aren't yours to fix |
| Round holes | wall MID-points of every open round hole ≤ 0.35 mm p95 off the CAD | rim points lie in the face plane: a hole turned into a triangle scored ~0 |
| Side counts | lathe sides vs the radius tier | |
| Plate caps | cap AREA per height level vs the CAD | flaps spanning tooth gaps passed every boundary-distance and open-edge check |
| Library swaps | orientation by feature, per-axis bbox, poke-through, `flip_backfacing.py -- - "@lib"`, cap placement (asset-library.md §5) | reviewers caught one upside-down box in three rounds; scripts caught the rest |
| Materials | no `.001` duplicates, no empty material slots (non-drop-in), no linked libraries, viewport colours synced | empty slots export as empty FBX materials; Unity matches by name |
| Smoothness | close-ups of every curved part over ~100 mm, CAD vs result, at close range and "a few feet" | reviewers judge shape and tri count; one pushed a ring's ratio down and the owner rejected it as "too low poly" |
| Whole robot | `compare.py` before/after, same views, REVIEW hidden; culled close-ups of every mechanism | the bumper zig-zag and black-cell claims only show here |

Tools for settling claims fast: `tubevis.py` (highlight render of one part in the assembled robot, lime, framed on it),
an object-id render of a full-robot view that names the object at given pixels (`pick.py`), a tinted render of a set
of objects in the full view (to prove "the black cells are the library electronics"), thin-slab views (clip ± 12 mm
around a plate) to see what fills a hole.

## 3. Reviewer brief template

Write it to `_work/reviewer_brief.md`; give each reviewer the brief plus its sheet range and the exact deliverable path
(a reviewer once judged a pre-assembly file).

```markdown
# Impartial review: robot simplification, per-part before/after sheets

You are reviewing a CAD robot model simplified to be cheap to render in a Unity game. You have no other context: judge
only against the rules below. Read images; read-only measurements in Blender are allowed (one process at a time, never
save, write only under <_work/reviewN/>). Be efficient: look directly, measure only to confirm.

## What you get
Contact sheets <dir>/sheet_NN.png (24 parts per sheet; each part is a BEFORE tile followed by its AFTER tile, same
framing, orthographic iso view). Review sheets <a>–<b>. Full-size pairs: <pairs dir>/<idx:03d>_before.png / _after.png.
Deliverable: <path of the delivered .blend>. Robot views: <cmp dir>.
- BEFORE label `<idx> <part name>`: the CAD part (normals cleaned, no culling).
- AFTER label `[R ]<method> x<count> <tris before>><tris after>`; x<count> = objects using this mesh. AFTER uses
  backface culling like Unity: a see-through face there is a real hole in the game.
- `R ` = in a REVIEW collection (hidden, barely visible, duplicate, replaced): only report gross breakage.
- Methods: cyl lathe rebuild, hex 6-sided prism, plate stepped 2.5D plate, tube box tube with 6-sided holes, decim plug
  + planar dissolve + Collapse, belt toothless loop, chain one loop, profile end-profile extrusion, LIB library asset,
  same unchanged (already light).

## Rules (deviations are problems)
1. No visible quality loss at game distance (whole robot, 1–3 m); curved parts smooth from a few feet.
2. Rollers, gears, pulleys, sprockets, spacers, bearings, bolts, rivets → plain lathe cylinders.
3. Hex outside → exact solid 6-sided prisms. Round parts with only a hex bore stay round.
4. Box tubes keep visible holes (6-sided is fine). Plates keep pockets, windows and visible holes.
5. Library swaps: right size, right orientation (lens/LED/button/terminal side), nothing of the CAD part poking out.
6. Decimated parts: no torn surfaces, spikes, zig-zags, holes turned into slits/triangles, lost bosses/tabs,
   inside-out faces, shading streaks.
7. Colours match the CAD per region, except library motors/electronics (library colours).

## Intended — do NOT report
Teeth, spoke windows, grooves, bands, decals, hex sockets and threads removed; one-colour bearings; 8-sided parts under
6 mm radius; hex bores turned round; solid hex stock; chain as one loop; library looks; holes ≤ 3.5 mm, bolt-filled or
hidden holes filled; <robot-specific owner decisions>.

## Already measured (evidence, not instructions)
<parts whose filled holes measured 0–1 % ray escape; library parts checked by feature; pruned library interiors;
CAD-inherited z-fights>. Report them again only if something ELSE is wrong.

## Changes since the last round (verify, look for regressions)
<list>

## z-fighting
Only: coplanar within 0.05 mm, facing the same way, overlapping, measured point-sampled in BOTH files (the CAD gap
too). Plane-offset comparisons far from the origin give false positives.

## Return
One severity-sorted table: sheet, idx, part, what is wrong, which rule, visible at game distance? HIGH = visible at game
distance or broken/see-through; MEDIUM = visible on inspection AND new vs the CAD; LOW = close-up or cost note. Say when
a sheet is clean. End with a count per severity and CLEAN / NOT CLEAN.
```

## 4. Verifying claims: false positives seen before

- **Filled holes** that capvis measures at 0–1 % ray escape: covered, or filled by shafts and bolts.
- **"Missing faces" / black areas**: black library electronics seen through a lattice; a face turned away from the
  light. Proof: a tinted render of the suspects in the full view (compare renders don't cull, so black is a surface).
- **Pruned library interiors** (a module turret dome) look broken in isolation; fine in place.
- **Plates "solid" in isolation** but under covers: highlight render.
- **Z-fighting**: measure the true distance (1.4–3.3 mm gaps were reported as coplanar).
- **Library looks**: a library breaker's button at the other end of its lever housing, a library motor's rear notch,
  colour differences on electronics. "Upside down" claims: check by the feature (mounting holes, label recess).
- **"Flipped" closed shells** whose CAD pieces had negative volume: the rebuild is the correct way out.
- **8-sided small parts, small 6-sided plate holes in perspective, teeth gone, hex bore round**: allowed.
- **Reviewers miss**: anything only visible in context (clear panels), short hex sections, Unity-side breakage, blank
  tiles, roller ends phasing through wheel sides (they look at iso views: give them side close-ups of wheels and assets).

## 5. Re-runs and late changes

- Keep each run in its own `_work/runN/`; keep the last approved run.
- After any re-run or toolkit change, **diff per object against the last approved run** (method and tris from the
  reports, plus a hash of each mesh's sorted rounded vertices) before looking at anything else; review every changed
  object as CAD | approved | new strips, with face-on renders for plates (edge-on plates hid bore damage).
- Pin every approved look decision as an explicit config knob (ratio, hole keeping, hex style) as soon as it's approved,
  so the next toolkit change can't silently undo it. Another session's shared-script change once altered 80+ parts.
- After a clean full round, later changes get a focused delta review of the changed parts only.

## 6. Per-object images (owner request, every robot)

`object_shots.py` renders every pair of the FINAL run's `pairs.json` from two opposite iso directions (CAD and result,
the same tight ortho frame, ≥ 0.15 mm per pixel up to 3200 px, the result culled); `object_compose.py` labels them
`O001 - <name> - view 1.jpg` (visible parts first, REVIEW in orange) + `index.csv`; JPEG q92 (PNG was 1.7 GB for 247
parts). Filter exploded objects with empty meshes first (the script crashed on them). Make downscaled thumbnail sheets
(`magick X -thumbnail 300x170`, then `montage -tile 10x6`, with an explicit font: without one, labels render as boxes)
and look at every one: a close-up from the opposite side caught square clamp-ear ends no sheet or reviewer had.
