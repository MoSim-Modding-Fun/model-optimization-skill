# The asset library

The team's low-poly models of vendor parts (motors, electronics, wheels, swerve modules, cameras, kit parts), shared by
every robot. A library swap is usually the biggest single saving on a robot, and it keeps robots consistent. Building
new assets: [building-assets.md](building-assets.md).

## Contents
1. Where it is, and the git policy
2. Layout and naming
3. Finding a match
4. Swapping it in
5. Orientation and placement checks
6. Colours
7. Known quirks of existing assets

## 1. Where it is, and the git policy

- GitHub: https://github.com/MoSim-Modding-Fun/Asset-Library (branch `main`; every `.blend` in **git LFS**; `*.blend1`
  ignored).
- Local clone: the path in the skill's config (`asset_library`, set by `install.py`; `python3 "$TK/paths.py" library` prints it). It holds `Components/`, `Materials/RobotMaterials.blend` (the shared finishes)
  and `Media/`.
- Before a job, check the clone is present and current: `git -C "<lib>" status --short`, `git -C "<lib>" rev-parse HEAD`
  vs `git -C "<lib>" ls-remote origin main` (both read-only). Untracked folders in `Components/` are assets someone built
  but never committed; they are usable, but say so.
- **Git policy:**
  - Run by the library maintainer (config `library_maintainer: true`): **ask before any git action that changes the
    library** (pull, add, commit, push).
  - Run by anyone else: **don't run git commands that change the library at all.** Build new assets where they say (or
    in the robot's `_work/`) and list them in the delivery so the maintainer can add them.
  - If you don't know which applies, ask.
- Never edit an existing library asset in place without a backup outside the library folder (`_work/lib_backup/`) and
  an OK (exception: a broken asset that nobody's sessions made may be fixed, after the backup).
- When you change a library asset that robots already use, tell every robot session that uses it; each must refit or
  rerun (a refit script on the finished file beats a 40-min rerun).

## 2. Layout and naming

- `Components/<part number> (<name>)/<part number> (<name>).blend`, e.g. `WCP-0940 (Kraken X60)/WCP-0940 (Kraken
  X60).blend`. Wheels carry size and durometer: `am-3945 (3in OD 1in WD Compliant Wheels)`. A "/" can't be in a folder
  name (use "-"). Kit parts may sit in a vendor subfolder under `Components/`.
- **The asset object or collection name is often NOT the folder name** (a breaker folder holds the object
  `am-0282 120A Breaker`; a flex-wheel folder spells "0.5in" where its object spells ".5in"; colour variants are
  separate objects such as `2.25in_green (…)`). `LIB_RULES` needs the object name: a folder name gave a KeyError and
  the wheels silently stayed CAD. List object names and tri counts with a small guarded script before writing the
  config.
- Library files also hold the source CAD (`HP` collection, excluded), `LP` / `Cutters` working objects and sometimes
  other assets (a motor file also holds a 1.4k-tri adapter board). Pick the object with `asset_data`, by exact name,
  never "the biggest mesh" or `"*"`.
- Object assets (one mesh, origin at the part) vs collection assets (swerve modules: collection → root empty →
  `<name> Parts` / `<name> Wheel` empties → parts). Module assets share one frame: the L-plate / bottom-plate outer
  corner at the origin, the module in −x / +y, z = 0 at the frame-rail top.
- Finishes are linked from `Materials/RobotMaterials.blend`. Outputs must not link the library: the pipeline runs
  `build.make_all_local()` before saving; check `[m.name for m in bpy.data.materials if m.library]` and
  `bpy.data.libraries` after any step that appends an asset.
- Some assets rely on modifiers (a battery is half a mesh plus a Mirror): always take the evaluated mesh.

## 3. Finding a match

- **Every** motor, electronics box, compliant/flex wheel and vendor swerve module gets checked against the library.
  Count the motors in the CAD and confirm each one was swapped ("how are there only 6 swaps when there's all motors").
- Match vendor part names first (`Kraken X60 Brushless Motor`, `Robot Battery`, `120A Main Breaker`). Anonymous meshes
  match by size **measured in the mesh's own frame** (world bboxes of rotated objects read up to √2 too big): roboRIO
  146 × 143 × 35 mm, Pigeon 2.0 47 × 45 × 13, CANivore 57 × 39 × 19, Ethernet switch 107 × 101 × 27, Limelight 4
  80.1 × 48.1 × 32.6 (the LL3 is 29.5 mm thick; Onshape names the LL4 "Minimal"), battery 180 × 76 × 167, signal
  lights 57 × 57 × 82 (domed) and 45 × 45 × 74 (tapered amber lens).
- A library model of a different product is not a match even if it fits under the limit (a signal light accepted at
  4.2 mm was the wrong product): compare sizes; a different product gets its own asset at the library's fidelity
  (show the A/B at the end).
- Never force a library model from another robot's config (one robot's "Limelight 3" setting was copied to a robot
  whose part was a Limelight 4): let the fit choose, then check.
- Wheels: same OD **and** width (vendor may differ), **and** the look must match: render CAD vs library in neutral
  grey (wheel axis = the bbox extent that differs from the other two; PCA is ambiguous for squat wheels) and have a
  fresh no-context subagent judge CLOSE ENOUGH / NOT. Check the face pattern, not only the OD (a 30A "stretch bore"
  wheel with trapezoid windows was widened where the CAD had arc slots). Not close enough → build a dedicated asset.
- Swerve modules: compare the per-piece result with the library module, in tris and in renders; check the revision by
  the sub-part names inside the module ("Lifted Turret" vs the plain turret) before choosing an asset.
- Library by default: if the library asset is heavier than the simplified CAD part, keep the library one and show
  both at the end with tri counts (`LIB_KEEP_HEAVIER = "all"`, the template default; those rows of the run's `report.json` carry
  `heavier_than_cad: [library tris, CAD tris]`, multi-piece swaps a "heavier … swapped anyway" log line).
- No match for a repeated vendor part → a new library asset (owner's habit), shown as an end decision with the
  per-robot fallback.

## 4. Swapping it in

| Mechanism | When |
|---|---|
| `LIB_RULES` (part name or mesh name → file, object) | single meshes: PCA + ICP over 48 starting orientations, accepted under 3 mm mean distance (`LIB_MAX_MM` per part when the CAD has wiring/labels the asset lacks, or a library signal light that includes its nut) |
| `LIB_ASSEMBLIES` | motors/gearboxes exported as many pieces; optional `"flip"`, `"max_mm"`, `"bbox_tiebreak"` |
| `LIB_COMBOS` | gearbox + motor fused into one CAD mesh |
| `LIB_ABSORB` | other-named pieces inside a swapped body (wheel hub inserts) → REVIEW |
| `LIB_IDENTITY` / `LIB_EMPTY_IDENTITY` | assets built in the part's or the assembly empty's own frame (no fit) |
| `LIB_IDENTITY_MOTORS` | library motors as separate children of a module asset (module assets have no motors) |
| `mk5n_place.py` / `mk5n_split.py` / `x2i_place.py` | library swerve modules: a new object per module, or the library parts put into the CAD's own objects |

After a module swap:
- **Reverse distance check**: robot pieces the asset doesn't cover must stay (a corner block living inside one robot's
  module assembly but outside the asset's).
- **Duplicates**: robot hardware coinciding with the placed asset (mounting screws, spacers the asset already has:
  sample each piece's surface, ≥ 60 % within 1.5 mm of the asset) → `REVIEW - Duplicates`.
- **Steer angle**: the placement "keep" test may keep the CAD wheels because their steer angle differs; the angle
  never matters, so move them to REVIEW.
- **A/B assignment** of mirrored module types: write frames for both assignments and compare the fit stats (the wrong
  one shows p90 ~26 mm and "keep" lines on every housing).

Motor caps: the library Kraken X60 has an open rear notch. A 20-tri cap (built from the library motor's own notch,
`"$TK/kraken_cap_asset.py"`) goes on **only** where the CAD motor had a cap: the pipeline measures how much of the cap's
notch-filling surface lies on CAD geometry (≥ 0.5 → cap) and logs `kraken cap check … coverage`.

## 5. Orientation and placement checks (after run 1, every robot)

The fit error ties within 0.1 mm across 180° / 90° turns of symmetric and near-square boxes, so check every library box
by a **feature**, not by eye:
- world centre of distinctive coloured faces, CAD vs result (breaker button, Limelight LED bar and lens, battery
  terminals, Pigeon label recess and hole diagonal);
- per-axis local bbox, CAD vs library, both faces (motors along their axis);
- renders of CAD and result from the same cameras framed on the CAD part.

Fix with `LIB_FLIP` (`"X"`, `"Y"`, `"Z"` = 180°, `"Z90"`, `"Z270"`, or a `+` sequence such as `"X+Z90"`, last token
first) and `LIB_NUDGE_MM`. The `LIMELIGHT_PARTS` path takes no flip: put a Limelight that needs one in `LIB_RULES`.

Known recurring offsets:
- The library Kraken X44 sits ~2.75–3.1 mm off along its axis on every robot so far (both faces shifted equally: a pure
  axial shift). Nudge it, or move it onto the CAD's mount face (measure the front face along the motor axis); use 0.3 mm
  short of a face it would otherwise be coplanar with.
- A library breaker can sit ~0.8–1.2 mm into its plate (shorter body, bases aligned).
- `LIB_NUDGE_MM` goes through the FIRST instance's rotation into shared mesh data: with instances facing different ways,
  shift in each object's local frame in a post step instead.

Also run `flip_backfacing.py -- - "@lib"` on the finished file (a Limelight asset once shipped with inward-wound LED
bores), and check that no CAD piece pokes through a library body (motor connectors).

## 6. Colours

- Motors and electronics keep the library's colours. The viewport colour, metallic and roughness are synced from each
  material's Principled BSDF (Unity's FBX import reads only the base colour and ignores metallic, so a white metallic
  "Steel - Machined" shows white). Merge `.001` duplicates (Unity matches materials by name).
- Wheels, treads and swerve modules take the robot CAD's colours (`LIB_CAD_COLOURS`, `build.recolour_from_cad`) unless
  the CAD colours are uninformative (no colour, export defaults, the whole module one colour): then keep the library's.
- A two-colour moulded wheel through the section lathe gets ONE colour (area majority); assign the tread by geometry
  (outer band, normals across the axis).
- Metallic finishes render near-black in Workbench; QA renders set metallic 0. A reviewer's "black part" is often just
  that, or black library electronics seen through a lattice.

## 7. Known quirks of existing assets

- The battery is stored upright; robots mount it flat, and it has fitted 180° off ("X" or "Y" flip after a check).
- Pigeon 2.0 and roboRIO: upside down or holes on the other diagonal (all four quarter turns fit within 0.01 mm).
- 120A breaker, Limelight 4 and Limelight 2+: fitted 180° off on several robots.
- The Limelight 3 and 3G share one CAD: force a 3G with `LIMELIGHT_PARTS = {part: "Limelight 3G"}`.
- The library MK5n turret has no dome (pruned, hidden under the housing in the asset pose): looks broken in isolation,
  fine in place.
- The library MK5n contains module-mounting screws and a corner spacer that robots also have as frame hardware
  (duplicates check above); it lacks a corner block some robots have inside the module.
- The 3-stage MAXPlanetary includes an output stage some CAD stacks don't have (a 90° gearbox instead): slide it along
  its axis onto the CAD's input face.
- The library Kraken X44 / X60 caps have smoothed chamfer normals (a cone-like gradient on the end caps); cosmetic.
- Some library meshes carry open edges (roboRIO, MAXPlanetary); invisible with culling, don't chase them.
- Two versions of the 3" compliant wheel exist on purpose (48-sided and 16-sided).
