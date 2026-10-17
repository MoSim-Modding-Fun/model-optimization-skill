---
name: simplify-robot-model
description: Turn a CAD robot (Onshape/SolidWorks .glb/.gltf, STEP, an imported .blend, or a game's own FBX) into a low-poly, game-ready model for MoSimulator (Unity) with the bundled Blender toolkit and the team asset library (github.com/MoSim-Modding-Fun/Asset-Library). Use this whenever someone wants a robot or mechanism CAD simplified, decimated, "optimized", made lighter or cheaper to render, cleaned up for Unity, or turned into a drop-in replacement for a robot prefab, and whenever a swerve module, wheel, motor, camera or other part should become a library asset, even if they only say "do the next robot", "here's the new CAD", "make this model cheaper" or "simplify all files in this folder".
---

# Robot model simplification (MoSimulator)

Goal: make a CAD robot as cheap to render in Unity as possible **without visible quality loss**, in the house style
(the owner's rulings). A job takes hours: inventory, 3–10 pipeline runs, several review rounds. Almost every expensive
mistake in past jobs came from assuming instead of asking, from widening or narrowing what was asked, or from a process
that ran out of memory.

`$SKILL` below is this skill's folder (the directory holding this file); `$TK` = `$SKILL/toolkit` (the Blender
scripts), `$S` = `$SKILL/scripts` (the resource guard). Read the reference files when a step points to them; don't read
them all up front.

## Ground rules

1. **Ask at intake, default afterwards.** Before any heavy work, ask every question whose answer you don't know
   (references/intake.md); don't fill gaps with guesses. After the plan, keep going: look trade-offs get the
   documented default and are shown all together at the end, with the triangle cost of each option and images of each.
   Asked at the moment, never defaulted: anything irreversible or outward-facing (deleting, writing into a Unity project
   or a repo, committing, pushing, downloading, waking other sessions), a discovery that changes the whole plan, and a
   request you think is wrong (say so plainly first).
2. **Do exactly what was asked.** "Every X" means every X; "a cap for this motor" doesn't mean every motor.
3. **Never modify the source file.** Work on copies; outputs go next to it. If the owner edited a previous output: back
   it up, diff it against your last delivery, ask about anything you can't explain, never restore what they removed.
4. **Never delete parts.** Hidden, duplicate, barely visible and library-replaced parts go into `REVIEW - …`
   collections for the owner's review.
5. **Guard every process and keep RAM free.** Blender, Python helpers, big greps, parsers, Unity: all through
   `$S/bl.sh` / `$S/guard.sh` (own memory-capped scope, machine-wide lock for heavy jobs, start gate, watchdog). No
   exceptions, not even a 2-second `--python-expr`. The guard's RAM check is never removed or loosened.
6. **Disk, not `/tmp`.** `/tmp` and Claude scratchpads may be RAM (check `df /tmp`). Blends, renders and QA output go in
   the robot's `_work/`.
7. **Shared toolkit and library.** Other sessions may run simplifications in parallel. Check ListAgents and the
   toolkit's git status before editing a shared script; announce it; never overwrite others' work.
8. **Git**: commit or push only with an OK; never a "Co-Authored-By: Claude" trailer.
9. **Not a drop-in** unless the owner says the result replaces the game's model directly.
10. **Finish every job with the line `SIMPLIFICATION DONE`** (owner's request).

## Before the first job on a machine

- The toolkit reads machine paths from `~/.config/simplify-robot-model/config.json` (written by `$SKILL/install.sh`) or
  environment variables (`$TK/paths.py`): `asset_library` (the clone of the asset library), `robot_config_dirs` (where
  your `r<name>.py` robot configs live), `robots_root`, `library_maintainer`. If the file is missing, run the installer
  or ask the person for the paths.
- **Who is running this?** `library_maintainer: true` means the library maintainer: owner rulings apply as stated, and
  library git changes are asked about. Anyone else: treat the owner rulings as the house style but confirm the taste
  rulings (budget, look trade-offs), and **don't run git commands that change the asset library**; hand new assets
  over. If the config doesn't say, ask.
- Needs: Blender 5.x on PATH, python3, ImageMagick (`magick`), systemd user scopes (for the guard), git + git-lfs for
  the library.

## Workflow

### 1. Intake: ask before running anything

Look cheaply first (file name, size and folder; earlier outputs; an existing robot config; other sessions), without
opening a big CAD yourself. Then ask, grouped into AskUserQuestion calls of up to 4 questions, your recommended default
first. The full list with the reasons is in [references/intake.md](references/intake.md); the core:

1. Which file is the source, is it final, and do I import it (big CADs: the owner usually imports)?
2. Where do outputs go (default: next to the source, never inside a Unity `Assets/` folder)? Did you edit an earlier
   output?
3. Which mechanisms move, which panels are clear, is the underside ever seen?
4. Anything to keep exactly (team numbers, lettering, logos, decorative patterns), anything you already simplified?
5. Swerve modules: vendor, revision, exactly the vendor's? Vendor CAD available? Library or robot-only asset?
6. Budget if not "as low as possible" (100k–500k visible tris, aim low); other sessions running; what to deliver and
   who pushes.

Don't ask settled things (references/owner-rules.md: swerve wheel angle never matters, bearings are one colour, hex
stock is solid, chains are one loop, library assets by default, …).

### 2. Setup

- Work dir `<robot folder>/_work/` on disk; one sub-folder per run (references/runs-and-delivery.md §1).
- Asset library: present and current (references/asset-library.md §1). `ls Components/` before deciding a part has no
  match.
- Find the input type in [references/input-types.md](references/input-types.md) (glTF soup, STEP, game FBX, -Y up, very
  large CAD, merged sub-assemblies) and follow its notes.
- New robot config: copy `$TK/robots/rexample.py` (or the closest existing config) to `<robot config dir>/r<name>.py`.

### 3. Inventory and plan (show it, then keep going)

`ROBOT=<name> $S/bl.sh "<source>" --python $TK/prep.py -- "<_work>"` writes the object list, world matrices and CAD
material names the checks need, welds, and splits merged sub-assemblies (`exploded.blend`). Then a guarded inventory
script: objects, unique meshes (shared datablocks), tris placed and unique, units and up axis, the floor (helper cubes
below the wheels → `FLOOR_Z`), the hierarchy and naming, transparent materials, the owner's modifier stacks, unwelded
soup, box-section sizes, the heaviest meshes with renders of the top ~40, every motor and electronics part with its size
and candidate library match, and the robot's lowest points (ground contact: keep the CAD).

Show the owner: totals, the biggest offenders, the planned method per group of parts, every library swap (and what has
no match), run-time estimate, the batched cleanup plan for a very large CAD, and the part-specific questions (second
intake round). Once those are answered, run; don't wait for approval of the plan itself.

### 4. Robot config

`<robot config dir>/r<name>.py`, selected with `ROBOT=<name>`. Fill it from the inventory: `part()`, `group_of()` and
`SEE_THROUGH_GROUPS`, `LIB_RULES` (asset **object** names, not folder names), `KEEP_AS_IS`, `EXPLODE`, `FLOOR_Z`,
`FORCE_METHOD` (keyed on mesh datablock names), `VIEWS`. Keep the template's defaults (`LATHE_REMODEL`, `SHADE_LATHE`,
one-colour bearings, `PRE_WELD_M`, caps only where the CAD had one). Every knob: [references/toolkit.md](references/toolkit.md) §3.

### 5. Runs

Order: prep → `vis.py` → `pipeline.py` → `implode.py` → `phase3.py` → robot-specific post steps (library modules
placed, chain loops joined, recolours, patches) → QA renders (commands: references/toolkit.md §1). Put a whole run under
ONE guarded call (`LOCK=1 MEM=5G $S/guard.sh bash _work/run.sh runN`, template in references/runs-and-delivery.md §3),
started with the Bash tool's `run_in_background`; wait for its notification or a done-file, never with a `pgrep -f` loop.
Never edit a script while it runs. Expect 3–10 runs; between runs, test fixes on 1–5 parts with a small script before a
full run, and patch small changes into the finished file instead of rerunning. After every run, diff it per object
against the previous one (`$TK/rundiff.py`). Lessons per part type: [references/gotchas.md](references/gotchas.md).

### 6. QA (mandatory)

[references/qa.md](references/qa.md): automated checks after every run (verify + tfcheck, totals from the final file,
open edges, back faces vs the CAD, hexcheck, capvis on every "holes closed" claim, point-sampled z-fighting, round-hole
fidelity, side counts, plate cap areas, library orientation by feature with `$TK/libcheck.py`, materials, smoothness of
big curved parts), then your own look at every pair, then fresh no-context reviewers (2–3 per round, a slice each, the
brief template in qa.md). Verify every claim before fixing; repeat until a round is clean. Per-object images at the end.

### 7. Decisions (at the end, or when the owner offers time)

Every look-changing default becomes one decision: options, triangle cost of each, in-context images with the same
cameras, a recommendation (the lighter option when the difference only shows close up; the look-keeping one for
identity features). Ask with AskUserQuestion and send the images. Map terse answers to the listed options and restate
the mapping in one line. Details: references/runs-and-delivery.md §4.

### 8. Deliver

`<name> - simplified.blend` (all source objects with names, parents and transforms; REVIEW collections; live shading
stack; self-contained) and `<name> - review/` (robot before-after, sheets, report.csv, totals, breakdown by method with
and without REVIEW, verify.txt, decisions, evidence, per-object images), plus `- cleaned.blend` for very large CADs and
the drop-in FBX only for a drop-in. The final message: numbers, decisions, new library assets, leftovers, what waits on
the owner, the reminder **"Unity: set the model's Normals import setting to Import"**, scratchpad cleared, and the last
line `SIMPLIFICATION DONE`. Details and git: references/runs-and-delivery.md §5–7.

### 9. Keep the skill current

New lessons go into this skill as lessons that stand on their own, without robot names or team numbers
(references/runs-and-delivery.md §8). Shared-toolkit default changes stay opt-in until tested on more than one robot.

## Resources and safety

The owner's machine: ~31 GB RAM with zram swap (also RAM), 24 threads, `/tmp` in RAM, often 2–4 robot sessions at once.
Memory pressure there once killed the whole Claude desktop app, every session with it: processes started without their
own scope count against the app.

- `$S/bl.sh <blender args>` for every Blender call (lock, 4 GB cap, 8 threads; `LOCK=0 MEM=2G` for short checks);
  `$S/guard.sh <cmd>` for everything else. Raise `MEM` only when `free -m` shows room. The guard waits until the job
  fits with ≥ 15 % of RAM still free and swap ≤ 40 %, and kills it above 90 % RAM or 60 % swap. It backgrounds the
  command (stdin heredocs don't reach it: write helper scripts to files); bound every loop that trusts a count read from
  a file.
- Before anything the guard doesn't cover (waking a session, a big build in another tool), check `free -m` yourself and
  wait while less than ~30 % of RAM is available.
- Keep tool output small (filter before printing).
- Other sessions: wake them one at a time after a RAM check, and archive advisors again before waking the next.
- Before a heavy job, hold off while `/proc/pressure/io` is high (the guard waits above 50 %).

## Reference files

- [references/intake.md](references/intake.md): the intake questions and the mistakes behind them.
- [references/owner-rules.md](references/owner-rules.md): every owner ruling, in their words, and the decision
  defaults.
- [references/input-types.md](references/input-types.md): glTF, STEP, game FBX, drop-ins, -Y up, very large CADs,
  merged and fused parts, block-outs.
- [references/asset-library.md](references/asset-library.md): the library, finding and swapping assets, orientation
  checks, colours, git policy.
- [references/building-assets.md](references/building-assets.md): new library assets (modules, wheels, cameras, motors,
  kit parts, accessories).
- [references/toolkit.md](references/toolkit.md): commands, what the pipeline does, every config knob, script catalogue.
- [references/qa.md](references/qa.md): checks, reviewer brief, false positives, re-run diffs, per-object images.
- [references/runs-and-delivery.md](references/runs-and-delivery.md): work layout, safe runs, templates, decisions,
  deliverables, final message, git.
- [references/gotchas.md](references/gotchas.md): lessons per part type and tool.
