---
name: simplify-robot-model
description: Turn a CAD robot (Onshape/SolidWorks .glb/.gltf, STEP, an imported .blend, or a game's own FBX) into a low-poly, game-ready model for MoSimulator (Unity) with the bundled Blender toolkit and the team asset library (github.com/MoSim-Modding-Fun/Asset-Library). Use this whenever someone wants a robot or mechanism CAD simplified, decimated, "optimized", made lighter or cheaper to render, cleaned up for Unity, or turned into a drop-in replacement for a robot prefab, and whenever a swerve module, wheel, motor, camera or other part should become a library asset, even if they only say "do the next robot", "here's the new CAD", "make this model cheaper" or "simplify all files in this folder".
---

# Robot model simplification (MoSimulator)

Goal: make a CAD robot as cheap to render in Unity as possible **without visible quality loss**, in the house style
(the owner's rulings). A job takes hours: inventory, 3–10 pipeline runs, several review rounds. Almost every expensive
mistake in past jobs came from guessing where a measurement or a check would have settled it, from widening or
narrowing what was asked, or from a process that ran out of memory.

`$SKILL` below is this skill's folder (the directory holding this file); `$TK` = `$SKILL/toolkit` (the Blender
scripts), `$S` = `$SKILL/scripts` (the resource guard). Read the reference files when a step points to them; don't read
them all up front.

## Ground rules

1. **Assume what the files and house rules settle; ask only for permissions and real blockers.** The owner's rule:
   "This should not be asked it should be assumed" (source, import, outputs, budget, moving and clear parts, the
   underside, kept features, swerve modules, delivery: references/intake.md §2 says how to decide each). State the
   assumptions in the plan, keep going, list them again at the end. Look trade-offs get the documented default and are
   shown all together at the end, with the triangle cost of each option and images of each. Asked at the moment:
   anything irreversible or outward-facing (deleting, writing into a Unity project or a repo you don't own, committing,
   pushing, downloading), a real blocker (no usable source; a request with two readings that make different jobs), and a
   request you think is wrong (say so plainly first).
2. **Do exactly what was asked.** "Every X" means every X; "a cap for this motor" doesn't mean every motor.
3. **Never modify the source file.** Work on copies; outputs go next to it. If the owner edited a previous output: back
   it up, diff it against your last delivery, keep their changes (they are on purpose) and never restore what they
   removed.
4. **Never delete parts.** Hidden, duplicate, barely visible and library-replaced parts go into `REVIEW - …`
   collections for the owner's review.
5. **Guard every process and keep RAM free.** Blender, Python helpers, big greps, parsers, Unity: all through
   `"$S/bl.sh"` / `"$S/guard.sh"` (own memory-capped scope, machine-wide lock for heavy jobs, start gate, watchdog). No
   exceptions, not even a 2-second `--python-expr`. The guard's RAM check is never removed or loosened.
6. **Disk, not `/tmp`.** On Linux `/tmp` and Claude scratchpads may be RAM (`df /tmp` says tmpfs). Blends, renders and QA output go in
   the robot's `_work/`.
7. **Shared toolkit and library.** Other sessions may run simplifications in parallel. Check ListAgents and the
   toolkit's git status before editing a shared script; announce it; never overwrite others' work.
8. **Git**: commit or push only with an OK; never a "Co-Authored-By: Claude" trailer.
9. **Not a drop-in** unless the owner says the result replaces the game's model directly.
10. **Finish every job with the line `SIMPLIFICATION DONE`** (owner's request).

## Before the first job on a machine

- The toolkit reads machine paths from `~/.config/simplify-robot-model/config.json` (written by `$SKILL/install.py`) or
  environment variables (`"$TK/paths.py"`): `asset_library` (the clone of the asset library), `robot_config_dirs` (where
  your `r<name>.py` robot configs live), `robots_root`, `library_maintainer`. If the file is missing, run the installer
  (`python3 "$SKILL/install.py"`) or ask the person for the paths.
- **Who is running this?** `library_maintainer: true` means the library maintainer: owner rulings apply as stated, and
  library git changes are asked about. Anyone else: the owner rulings are the house style (they may override them),
  and **don't run git commands that change the asset library**; hand new assets over. If the config doesn't say, ask.
- **Windows, macOS and Linux.** Needs: Blender 5.x (found on PATH or in the usual install folder, or set as `blender` in
  the config), Python 3.8+, ImageMagick 7 (`magick`), git + git-lfs. Commands in this skill are bash: on Windows Claude
  Code runs them in Git Bash, so `"$S/bl.sh"` works there too; from cmd/PowerShell use `scripts\bl.cmd` /
  `scripts\guard.cmd`. `python3` in commands means your Python 3: on Windows `python3` is usually the Microsoft Store
  stub, so run `python` there (under the guard, a bare `python3` already means the guard's own Python). Quote every
  path: Windows user folders often have spaces. The guard caps memory with systemd scopes on Linux, job objects on
  Windows and a polling watchdog on macOS (`"$S/guard.py"`); Windows notes in
  [references/runs-and-delivery.md](references/runs-and-delivery.md) §2.

## Workflow

### 1. Intake: look, then assume

Look first (file name, size and folder; earlier outputs and owner edits; an existing robot config; other sessions).
Then assume, don't ask ([references/intake.md](references/intake.md) §2 has how to decide each):

- **Source and import**: the file given, final; if nothing provided is a `.blend`, import it yourself.
- **Outputs and delivery**: next to the source, the default deliverables, not a drop-in, nothing pushed without an OK.
- **Budget**: always the minimum tris with no visible loss.
- **Moving and clear parts**: from the hierarchy, names and materials; the underside is seen.
- **Kept features**: find bumper numbers, lettering, logos and decorative patterns yourself and keep them sharp; the
  owner's own simplified parts stay.
- **Swerve modules**: vendor and revision from the part names, confirmed by fitting the library module; library if it
  fits, else robot-only (A/B at the end).
- **Everything else settled** in references/owner-rules.md (wheel angle, one-colour bearings, solid hex, chain loops,
  library assets by default, …).

Ask only for permissions (downloads, commits, pushes, deletions, writing outside the robot folder) and real blockers.

### 2. Setup

- Work dir `<robot folder>/_work/` on disk; one sub-folder per run (references/runs-and-delivery.md §1).
- Asset library: present and current (references/asset-library.md §1). `ls Components/` before deciding a part has no
  match.
- Find the input type in [references/input-types.md](references/input-types.md) (glTF soup, STEP, game FBX, -Y up, very
  large CAD, merged sub-assemblies, every copy of a part merged into one mesh) and follow its notes.
- New robot config: copy `"$TK/robots/rexample.py"` (or the closest existing config) to `<robot config dir>/r<name>.py`.

### 3. Inventory and plan (show it, then keep going)

`ROBOT=<name> "$S/bl.sh" "<source>" --python "$TK/prep.py" -- "<_work>"` writes the object list, world matrices and CAD
material names the checks need, welds, and splits merged sub-assemblies (`exploded.blend`). Then a guarded inventory
script: objects, unique meshes (shared datablocks), tris placed and unique, units and up axis, the floor (helper cubes
below the wheels → `FLOOR_Z`), the hierarchy and naming, transparent materials, the owner's modifier stacks, unwelded
soup, box-section sizes, the heaviest meshes with renders of the top ~40, every motor and electronics part with its size
and candidate library match, and the robot's lowest points (ground contact: keep the CAD).

Show the owner: totals, the biggest offenders, the planned method per group of parts, every library swap (and what has
no match), the kept identity features, run-time estimate, the batched cleanup plan for a very large CAD, and an
"Assumed" list (moving groups, clear parts, module and part identities, anything else you decided). Then run; don't wait
for approval of the plan or answers to the assumptions: the owner corrects them if needed.

### 4. Robot config

`<robot config dir>/r<name>.py`, selected with `ROBOT=<name>`. Fill it from the inventory: `part()`, `group_of()` and
`SEE_THROUGH_GROUPS`, `LIB_RULES` (asset **object** names, not folder names), `KEEP_AS_IS`, `EXPLODE`, `FLOOR_Z`,
`FORCE_METHOD` (keyed on mesh datablock names), `VIEWS`. Keep the template's defaults (`LATHE_REMODEL`, `SHADE_LATHE`,
one-colour bearings, `PRE_WELD_M`, caps only where the CAD had one). Every knob: [references/toolkit.md](references/toolkit.md) §3.

### 5. Runs

Order: prep → `vis.py` → `pipeline.py` → `implode.py` → `phase3.py` → robot-specific post steps (library modules
placed, chain loops joined, recolours, patches) → QA renders (commands: references/toolkit.md §1). Put a whole run under
ONE guarded call (`LOCK=1 MEM=5G "$S/guard.sh" bash _work/run.sh runN`, template in references/runs-and-delivery.md §3),
started with the Bash tool's `run_in_background`; wait for its notification or a done-file, never with a `pgrep -f` loop.
Never edit a script while it runs. Expect 3–10 runs; between runs, test fixes on 1–5 parts with a small script before a
full run, and patch small changes into the finished file instead of rerunning. After every run, diff it per object
against the previous one (`"$TK/rundiff.py"`). Lessons per part type: [references/gotchas.md](references/gotchas.md).

### 6. QA (mandatory)

[references/qa.md](references/qa.md): automated checks after every run (verify + tfcheck, totals from the final file,
open edges, back faces vs the CAD, hexcheck, capvis on every "holes closed" claim, point-sampled z-fighting, round-hole
fidelity, side counts, plate cap areas, library orientation by feature with `"$TK/libcheck.py"`, materials, smoothness of
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

- `"$S/bl.sh" <blender args>` for every Blender call (lock, 4 GB cap, 8 threads; `LOCK=0 MEM=2G` for short checks);
  `"$S/guard.sh" <cmd>` for everything else. Raise `MEM` only when free memory allows (`python3 "$TK/sysmem.py"`). The
  guard waits until the job fits with ≥ 15 % of RAM still free and swap ≤ 40 %, and kills it above 90 % RAM or 60 % swap
  (Windows: 97 % of the commit limit). Write helper scripts to files in `_work/` rather than heredocs, so a rerun is
  the same command; bound every loop that trusts a count read from a file.
- Before anything the guard doesn't cover (waking a session, a big build in another tool), check free memory yourself and
  wait while less than ~30 % of RAM is available.
- Keep tool output small (filter before printing).
- Other sessions: wake them one at a time after a RAM check, and archive advisors again before waking the next.
- Before a heavy job on Linux, hold off while IO pressure is high (the guard waits above 50 %).
- **A simplification is usually not the priority job on the machine.** When the owner names another session's work as
  the priority (a model training, a game build), run every guarded job with `YIELD_TO='<its process pattern>'
  MIN_FREE=40 MAX_SWAP=25`: nothing starts while a matching process over 500 MB runs, and a running job is killed
  (exit 76, rerun it later) when one starts. Don't spawn reviewer agents while it runs. Another session's unscoped
  13 GB training run plus a 4 GB game build once filled the Claude app's cgroup with 22 GB of swap, and systemd-oomd
  killed every session at once, though no simplification job was running. If you see such a process, tell its
  session that a separate scope (`systemd-run --user --scope -p MemoryMax=…`) limits a crash to that one job.

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
