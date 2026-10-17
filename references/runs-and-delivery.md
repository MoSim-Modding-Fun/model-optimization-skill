# Runs, decisions and delivery

How to organise the work on disk, run it safely, present decisions and deliver. `$TK` = the skill's `toolkit/`,
`$S` = the skill's `scripts/` (guard.sh, bl.sh).

## Contents
1. Work folder layout
2. Running jobs safely
3. A run script and an assemble script
4. Owner decisions
5. Deliverables
6. The final message
7. Git
8. Keeping the skill current

## 1. Work folder layout

```
<robot folder>/
  <source>.blend / .glb / .step           never modified
  <name> - cleaned.blend                   very large CADs only (Phase 1)
  <name> - simplified.blend                the deliverable
  <name> - review/                         review folder (section 5)
  _work/
    guard.sh, bl.sh                        copies of $S (or call them from the skill)
    run.sh, assemble.sh, qa.sh             this robot's chains (section 3)
    vis.json, orig_names.json, tf_src.json, cad_mats.json, exploded.blend, src_zup.blend
    runN/                                  one folder per pipeline run: blends, report.json, logs, run.done
    qa_before/                             "before" renders, made once
    reviewN/                               reviewer briefs and their outputs
    decisions/                             option files, images, decisions.md
    lib_backup/                            anything you overwrite in the library, before you overwrite it
    simplified_backup_MMDD_HHMM.blend      the previous deliverable, before every assemble
```

Everything big lives in `_work/` on disk; on Linux `/tmp` and the session scratchpad may be RAM (`df /tmp`: tmpfs). A
12–13 GB scratchpad once froze the whole desktop. `_work` grows ~0.5 GB per run; delete superseded runs' renders at
delivery if space matters. Keep every helper script you write in `_work/` with a one-line usage header.

## 2. Running jobs safely

- **Every process through the guard**: `"$S/bl.sh" <blender args>` (lock, 4 GB cap, 8 threads by default),
  `LOCK=0 MEM=2G "$S/guard.sh" python3 helper.py ...` for helpers. Never `blender` directly, not even a 2-second check.
- **A whole chain under ONE guarded call**: `LOCK=1 MEM=5G "$S/guard.sh" bash _work/run.sh runN`. Each separately
  guarded step waits in the shared queue again (the lock is not first-come-first-served), and a guarded script started
  inside a guarded batch waits forever for the lock its parent holds. Short steps (renders, implode, small checks) can
  run `LOCK=0` under the RAM gate.
- **Detach long runs** with the Bash tool's `run_in_background` (any OS; on Linux/macOS also `setsid nohup … & disown`
  with a done-file and an exit-code file): session restarts kill plain `&` jobs. Wait by PID (`while kill -0 $PID; do sleep 20; done`) or on a
  `run.done` file, never with `pgrep -f` / `pkill -f` on a pattern that appears in your own command line (it matches
  itself: a wait loop sat 3 hours after the job finished, and a `pkill` killed its own shell). Anchor patterns
  (`^blender -b .*runN/`). Delete old done-markers and logs before relaunching (`tail -F` prints the OLD file first).
- **Never edit a shell script while it runs**: bash reads scripts as it goes and resumes mid-line. Stop it and run the
  remaining steps by hand.
- After a machine suspend, wall-clock timers (`timeout`, log seconds) mislead; check the process's CPU ticks before
  calling it hung.
- A running Blender has already imported the toolkit: edits by another session affect only jobs started later; a job
  launched during someone's edit can load a half-written file (check `python3 -m py_compile` first).
- **Windows (Git Bash)**:
  - Paths: forward slashes (`C:/Users/…` or `/c/Users/…`), always quoted (user folders often have spaces); stay under
    260 characters (deep robot folders + long part names).
  - Python: `python3` is usually the Microsoft Store stub (exit 9009, "Python was not found"); use `python` or `py -3`.
    Under the guard a bare `python3`/`python` runs the guard's own Python, a bare `bash` is Git's bash (on a machine
    with WSL, `bash` from Windows' own folder would start Linux instead), and a `.sh` script runs through that bash.
  - Blender is usually not on PATH: the guard and `"$TK/paths.py" blender` use the configured one.
  - File names can't contain `: * ? " < > |`, end in a dot or space, or be CON/NUL/COM1…, and object names often break
    these rules (`paths.safe_filename`). Names that differ only in case land in one file (also on macOS).
  - Keep shell scripts LF (the repo's `.gitattributes` does; CRLF breaks bash).
  - The memory cap is a job object: it counts committed memory, a little more than the resident size Linux counts, and
    an allocation past it fails, so Blender exits with a crash code; the guard reports that as 137 ("reached its cap").
    If a job that fits on Linux dies at the cap here, raise `MEM` by ~1 GB when free memory allows.
  - Interrupting a Bash call may not reach the guard there (it can run outside Claude Code's process tree): after you
    stop a guarded job, check `tasklist | grep -i blender` and end leftovers with `taskkill //T //F //PID <pid>`.
- **macOS**: there's no hard per-job cap; the guard polls the job's memory footprint every 2 s and kills it past `MEM`,
  so a sudden spike can briefly overshoot. "Available" is the memory-pressure level, swap is measured against RAM.
- The shell may be zsh (the owner's Linux machine): no `mapfile`, `echo ===` fails, unmatched globs abort the command, `"$B:Assets/…"` expands
  `:A` (write `"${B}:Assets/…"`). `rm -rf $VAR/...` is blocked by the safety check: literal paths or `"${VAR:?}"`.
  The owner's interactive shell is fish: heredocs in commands you hand them fail; use `-m` flags.

## 3. A run script and an assemble script

```bash
#!/bin/bash
# _work/run.sh <run>: pipeline -> post steps -> run.done. Run it as: LOCK=1 MEM=5G guard.sh bash _work/run.sh run3
set -u
TK=<skill>/toolkit; W="<robot folder>/_work"; O="$W/$1"
rm -rf "${O:?}"; mkdir -p "$O"
export ROBOT=<config name>
for PY in python3 python py; do "$PY" -c 'import sys; sys.exit(sys.version_info < (3, 8))' 2>/dev/null && break; done
BLENDER="$("$PY" "$TK/paths.py" blender)"        # the configured Blender (any OS; Windows' python3 is a stub)
B() { "$BLENDER" -b -t 8 "$@"; }                # already inside the guard's cap
B "$W/exploded.blend" --python "$TK/pipeline.py" -- "$W/vis.json" "$O/simplified_exploded.blend" "$O/report.json" > "$O/pipeline.log" 2>&1 &&
B "$O/simplified_exploded.blend" --python "$TK/implode.py" -- "$O/imploded.blend" > "$O/implode.log" 2>&1 &&
B "$O/imploded.blend" --python "$TK/phase3.py" -- "$O/phase3.blend" "$O/phase3.json" --cad-mats "$W/cad_mats.json" > "$O/phase3.log" 2>&1 &&
# robot-specific post steps here (library modules placed, chain loops joined, recolours, patches), each reading the
# previous step's file so one step can be rerun alone on a finished run
date > "$O/run.done"
```

Chain with `&&`: a killed pipeline otherwise lets later steps run on stale files and write `run.done`.

Multi-piece library fits cost ~40 s each (48 PCA/ICP starts on the joined point cloud): 28 of them (motors, encoders,
cameras, electronics) were most of a 27-minute pipeline, and twice that while another session trained a model on most
cores. Batch the fixes of one QA pass into the next run, and stop a run that a pending fix will redo anyway.

The assemble script (deliverable + review folder):
1. Back up an existing deliverable to `_work/simplified_backup_MMDD_HHMM.blend`, and refuse to continue if it changed
   since your last delivery (the owner edited it) until you've diffed it.
2. Restore anything the run changed for its own convenience (a rotated root).
3. Merge `.001` materials into their originals when the colours match; sync every material's viewport colour, metallic
   and roughness from its Principled BSDF (assets appended after the pipeline keep 0.8 defaults); drop unused material
   slots (not in a drop-in); `build.make_all_local()`; drop empty REVIEW collections; set faces smooth under every
   Smooth by Angle stack; `save_version = 0`; save compressed.
4. Review folder: renders, sheets, `report.py`, `verify.py` + `tfcheck.py` + `hexcheck.py` into `verify.txt`, the
   decision images, the evidence images, the per-object images.

## 4. Owner decisions

- Collect every look-changing default during the run (a part becoming a primitive, a hole pattern dropped or kept,
  library vs per-piece module, a heavier library asset kept, a decorative feature flattened, a colour choice that
  changes how the robot reads, spring style). Don't batch what changes the build's source or scope (module source,
  wheel source, cap placement): those were intake questions.
- One decision = options A/B(/C), the triangle cost of each, images of each option **in context**: the part opaque and
  everything else at ~50 % opacity, or a camera whose near plane cuts away the walls in front (find the directions it is
  visible from with `tubevis.py` first), the same cameras for every option, each option rendered in its own Blender
  process (swapping option B in one decision changes the scene behind the next). Hide panels that are clear in game.
  Keep baked custom normals when judging them (`KEEP_NORMALS=1` for `decision_shot.py`).
- Mark the default "(Recommended)": the lighter one when the difference only shows close up, the look-keeping one for
  identity features (owner-rules.md §5).
- Write `decisions/decisions.md` (options, costs, recommendation; later the answers and how to switch), and send small
  JPGs (≤ 1 MB; 7–8 MB PNGs timed out) with the question. Ask with AskUserQuestion.
- Apply answers by patching the reviewed file (pipeline-identical code on the affected parts) rather than a full rerun,
  then pin them in the config.

## 5. Deliverables

Next to the source, unless told otherwise:
- `<name> - simplified.blend`: every source object with its name, parent and world matrix; REVIEW collections
  (`REVIEW - Hidden`, `- Hidden behind other mechanisms`, `- Barely visible`, `- Duplicates`, `- Replaced by library`);
  live Smooth by Angle → WeightedNormal; self-contained (no linked libraries); compressed.
- `<name> - cleaned.blend` for very large CADs. A drop-in FBX (plus an export-ready baked `.blend`) only when it's a
  drop-in.
- `<name> - review/`:
  - `robot before-after.png` (10–15 views, CAD | result, REVIEW hidden; a small JPG too, for sending);
  - `per-part sheets/`;
  - `report.csv` (object, part, mesh, method, tris before / after, REVIEW collection, note) and `totals.json`;
  - `triangle breakdown.md` (by method; totals with and without REVIEW; per REVIEW collection; stage totals for big
    CADs);
  - `verify.txt`;
  - `decisions/` (images + `decisions.md`);
  - `evidence/` (highlight renders behind every "hidden" claim, library orientation checks, capvis JSONs, before/after
    of any fix the owner asked for);
  - `objects/` (per-object images + `index.csv`).
- New library assets, listed separately (with tris and images).
- Notes for this robot (decisions, numbers, anything odd) in the robot's own notes, outside the skill.

## 6. The final message

Files and paths; totals (source → cleaned → simplified; visible, with REVIEW, per REVIEW collection; drawn in game for a
drop-in); the by-method table; decisions with costs (images sent); new library assets; known leftovers (lows); the
reminder **"Unity: set the model's Normals import setting to Import, or the weighted normals are thrown away"**; what
waits on the owner (REVIEW review, decisions, pushes); scratchpad cleared; and as the very last line:

```
SIMPLIFICATION DONE
```

## 7. Git

- Commit or push only with an OK, in any repo. No "Co-Authored-By: Claude …" trailer.
- The asset library: asset-library.md §1.
- A robot repository with several sessions: run git under a shared lock (`"$S/guard.sh" --lock --lock-name git -- git …`); one
  commit per robot for the models and review folder, then `_work` in parts (≤ ~1,500 files / 1.5 GB each); `.blend`,
  `.glb`, `.gltf`, `.step` in LFS, while FBX and images may not be (GitHub rejects files over 100 MB outside LFS); leave
  out caches (a throwaway Unity project's `Library/`), raw renders and `*.blend1`.
- The harness may block a commit that sweeps in other sessions' work: give the owner the exact `git add` paths and
  message instead.

## 8. Keeping the skill current

When a job teaches something new (a ruling, a gotcha, a tool fix), write it into this skill as a lesson that stands on
its own, with no robot names or team numbers. Changes to shared toolkit defaults are opt-in config knobs until tested
on more than one robot (a default that helped one robot once made another worse). Announce edits to the toolkit to the
other active sessions first (a short "LOCK <file>" message, then "done"), merge function by function into the current
files, and never copy a private snapshot over shared files.
