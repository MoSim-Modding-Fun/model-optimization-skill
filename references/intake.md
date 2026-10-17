# Intake: assume what you can, ask only what you must

The owner's rule: everything that the files, measurements or house rules can settle is **assumed, not asked**
(2026-10-16, about swerve modules, kept features, budget, delivery, moving and clear parts, the underside and importing:
"This should not be asked it should be assumed"). Asking those costs the owner time and reads as not having looked.
State every assumption once in the plan message (a short "Assumed" list), carry on, and list them again in the final
summary so the owner can correct any of them. Ask only for permissions and real blockers (§3).

## Contents
1. Look first
2. Assume (and how to decide)
3. Ask only these
4. Drop-ins (only when told it replaces the game's model)

## 1. Look first

- The input: name, size, extension, folder, modified date; a `.glb` header (the JSON chunk) for a hierarchy and tri
  summary.
- The robot folder: an earlier `- simplified.blend` (a `.blend1` next to it means the owner saved edits), a `_work/`, a
  review folder, a robot config, notes from an earlier session.
- Other files in the folder: a second `.blend` may be a different robot design, not a revision; files no prefab uses may
  still be wanted.
- Other sessions working right now (ListAgents / list_sessions): shared heavy-job lock, shared toolkit edits, RAM.

## 2. Assume (and how to decide)

| Topic | Assume | How to decide |
|---|---|---|
| Import | If nothing provided is a `.blend`, import it yourself. If the owner says they will import, wait. | input-types.md §2–4 |
| Source and outputs | The file given is the source and final; outputs go next to it (`<name> - simplified.blend` + `<name> - review/`; never inside a Unity `Assets/` folder). Read game files with `git show`, never switch someone's branch. Not a drop-in. | |
| Delivery | The default set (SKILL.md step 8). Nothing committed or pushed without an OK (asked at the end). Past robot sessions are woken for advice only when something is stuck. | |
| An earlier output | If the owner edited a previous output, build on their saved file: back it up, diff it, keep their changes. | `.blend1`, mtime, object diff |
| Budget | Minimum tris with no visible loss ("should always be min tris"). Never ask for a target. | |
| Moving parts | Rigid groups from the hierarchy and names: each swerve module its own group; intake, arm, turret, hood, shooter pivot, climber, elevator stages, slides; a game prefab's joint bodies. When unsure, put a part in a moving group: a moving group never hides chassis holes, so the mistake costs a few tris, not a hole. | hierarchy, names, pivots |
| Clear parts | Materials or names saying transparent, clear, polycarbonate, polycarb, Lexan, acrylic, glass; alpha < 1 or transmission > 0; clear renderer materials in a game prefab. Put them in a see-through group. Export colours often hide clear parts: check names too. | materials, names |
| The underside | Seen: holes and parts underneath count as visible (`NO_FLOOR_ALL = True`). | |
| Identity features | Keep them: bumper numbers and lettering, engraved or raised text, logos, decorative patterns (swirls, truss pockets). Find them yourself: bumpers almost always carry the team number; look for text-like material regions or dense small raised/engraved detail on bumpers, covers and printed parts in the inventory renders. Keep them sharp (protected Collapse or `KEEP_AS_IS`); their cost goes in the end decisions. | renders, materials, small-feature density |
| Owner-simplified parts | Parts carrying Decimate → Smooth by Angle → WeightedNormal stay as they are. A part already named "Simplified …" with a sane tri count is treated as already light. | modifiers, names |
| Swerve modules | Identify vendor and revision from the part names inside the module (e.g. "SDS MK5n", "MK5 Lifted Turret", "MK4n", "X2i") and confirm by fitting the library module on the housing (median distance well under 1 mm). Fits → library module, robot CAD colours (unless uninformative). Doesn't fit (another revision, custom parts) → a robot-only module asset or the per-piece result, shown as an A/B at the end. Wheel steer angle never matters. | names, module fit |
| Electronics and anonymous parts | Match by size in the mesh's own frame and by fit (asset-library.md §3); orientation by feature. Limelight 3 vs 3G (same case): decide from the CAD's front (LED count and layout, lens barrel). | sizes, fits, renders |
| Wheels | Library wheel of the same OD and width that looks like the CAD (no-context judge); otherwise build one. Colours from the CAD unless uninformative. | sizes, renders |
| Stock sizes, up axis, floor | Measure: metric vs imperial tubes, hex stock size, the up axis (deliver in the source's orientation), helper cubes below the wheels (to REVIEW), ground contact (keep the CAD as it is). | inventory |
| Bodies missing from the export | Keep what the CAD shows; note it in the summary (and as a decision if a library part could stand in). | name vs size |
| Very large CAD | Run the batched cleanup first; state the plan and the time. | tri counts |
| Other sessions | Check them yourself; coordinate toolkit edits and heavy jobs with the active ones. | ListAgents |

## 3. Ask only these

- **Permissions**, at the moment they come up: installing or downloading anything (a STEP importer venv), committing or
  pushing (ask at the end), deleting files, writing outside the robot folder (a Unity project; the asset library when
  you aren't its maintainer).
- **Real blockers**: no usable source (several unrelated candidate files and the request doesn't say which; a broken or
  empty export), or a request whose two readings would produce different jobs.
- **Look trade-offs** are not intake questions: default them and show them at the end (SKILL.md step 7).

## 4. Drop-ins (only when told it replaces the game's model)

The default is **not** a drop-in. When the owner says the result will overwrite the game's FBX under its prefab, assume:
the prefab is not edited ("just make it not have missing meshes"); deliver both the drop-in FBX and an export-ready baked
`.blend`; moving and clear parts come from the prefab's joints and renderer materials. Tell the owner up front (not as a
question) that REVIEW objects keep rendering in game and that deleting them later can't hide objects sharing a mesh with
kept ones, and list which REVIEW objects share meshes before they review.
