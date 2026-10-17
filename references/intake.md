# Intake: what to ask, and why

Intake is where "ask over assuming" happens. After it, the job runs on defaults and every look trade-off waits for the
end. So ask everything you don't know now, in as few AskUserQuestion calls as possible (≤ 4 questions each, with
options and your recommended default first). Skip a question only when the request or the files answer it for
certain. Each question below exists because a past job guessed wrong.

## Contents
1. Before asking: look cheaply
2. Round 1, before any Blender work
3. Round 2, with the plan (after the inventory)
4. Drop-in questions (only when told it replaces the game's model)
5. Never ask (settled)

## 1. Before asking: look cheaply

- The input file: name, size, extension, folder, modified date. A `.glb` header (the JSON chunk at the start) is fine to
  read for a hierarchy and tri summary; don't import or parse a big CAD's buffers yourself.
- The robot folder: an earlier `- simplified.blend` (a `.blend1` next to it means the owner saved edits), a `_work/`, a
  review folder, a robot config (`r<name>.py` in the configured robot-config folders), notes from an earlier session.
- Other files in the folder: a second `.blend` may be a different robot design, not a revision; files no prefab uses
  may still be wanted ("simplify all files in this folder").
- Is the file identical to one in the game project (`cmp`, or `git hash-object` vs `git ls-tree -l`)? That makes it the
  game's own model. It does NOT make the job a drop-in (default: not a drop-in).
- Other sessions working right now (ListAgents / list_sessions): shared heavy-job lock, shared toolkit edits, RAM.

## 2. Round 1, before any Blender work

| Question | Why |
|---|---|
| Which file is the source, and is it final? | Input paths have vanished mid-job when a branch was switched; exports get replaced. Read game files with `git show`, never switch someone's branch. |
| Should I import the `.glb` / `.gltf` / STEP, or will you? For STEP: may I install an importer (OpenCASCADE in a venv under `_work/`)? | The owner imports big CADs themselves unless the request says otherwise; installing packages is a download that needs an OK. |
| Where do outputs go? (default: next to the source; never inside a Unity `Assets/` folder) | Unity imports anything written into `Assets/`. |
| Did you edit an earlier output? Build on it or start over? | Work on top of the owner's saved file, backed up and diffed first. |
| Which mechanisms move? (offer the rigid groups you can see in the hierarchy) | `group_of` drives the visibility pass; a part hidden only behind a moving mechanism isn't hidden. Guesses from names ("Moving <1>") and from other robots have been wrong both ways. |
| Which panels are clear? (offer parts with transparent/polycarbonate/Lexan materials) | Clear parts must never hide anything; without this, dozens of parts behind clear panels went to REVIEW and visible tube holes were dropped. Export colours lie about clear parts. |
| Is the underside ever seen (climbing or tipping robot, an under camera)? | The floor counts as an occluder for hole culling by default; holes on rails' undersides were dropped and later wanted. |
| Anything you already simplified, or want kept exactly (team numbers, lettering, logos, decorative patterns)? | `KEEP_AS_IS`; identity features beat the part-type rules. |
| Budget different from "as low as possible, no visible loss at a few feet"? | Default 100k–500k visible tris, aim low. |
| Swerve modules: which vendor and revision? Exactly the vendor's module (L-plate configuration, through-frame fasteners, wheel)? Can you supply the vendor CAD? Library asset, robot-only asset, or both? | Robot CADs are often an older or customised revision; a library module of the wrong revision was fitted before anyone checked. |
| Other robot sessions running? Should I ask the past robot sessions for advice? | RAM and the shared lock; advisors are woken one at a time. |
| What do you want at the end, and who commits/pushes? | Defaults: SKILL.md step 8. Pushes need an OK; the harness may block them, then prepare the commands. |

## 3. Round 2, with the plan (after the inventory)

Ask about specific parts the inventory couldn't settle, with sizes and thumbnails, in the same message as the plan.
Once answered, keep going.

- Anonymous boxes and cameras: "`mesh850_mesh` is 146 × 143 × 35 mm: roboRIO?" Which Limelight model (3, 3G, 4, 2+)?
  A fit can't tell a 3 from a 3G (same case); an anonymous "Minimal" part has always been a Limelight 4 by size.
- Library parts that differ in size or model from the CAD (a signal light 57 mm vs the CAD's 45 mm): use the library
  one or build a new asset?
- Wheels with no library match, or whose look doesn't match (a new library wheel will be built).
- Stock sizes when ambiguous: metric 25 × 50 mm vs imperial 1 × 2 in tubes (metric tubes lost every hole once).
- Up axis and the delivered orientation when the CAD isn't Z up ("orientation can be either way" was one answer).
- The floor: Onshape origin/reference cubes hanging below the wheels; wheels slightly above z = 0 (keep the CAD).
- Very large CAD: confirm the batched cleanup plan and the time it will take.
- Bodies the export seems to be missing (a "3 in" wheel that measures 38 mm: only the hub came through).
- Anything where two readings of the request are possible.

## 4. Drop-in questions (only when told it replaces the game's model)

The default is **not** a drop-in. Only if the owner says the result will overwrite the game's FBX under its prefab:

- Which prefab and branch? (read with `git show`; never switch the branch)
- May the prefab be edited? (default no: "just make it not have missing meshes")
- Will you export the FBX yourself from the `.blend`, or use the FBX I deliver? (live modifiers break a plain export:
  deliver an export-ready baked `.blend` too)
- Should hidden/duplicate parts stop rendering in game? Explain first: REVIEW objects stay in the FBX and keep
  rendering; deleting them later can't hide objects that share a mesh with kept ones. Tell the owner which REVIEW
  objects share meshes BEFORE they review.
- Which mechanisms move in game, and which panels are clear in game? (read the prefab's joints and materials, then
  confirm)

## 5. Never ask (settled)

- Swerve wheel steer angle (never matters); bearing colours (one colour); hex stock bores (solid); chain style (one
  loop); motor caps (only where the CAD had one); lathe grooves (one plain cylinder); compliant/flex wheels (library,
  same OD × width); library vs lighter CAD part (library, A/B at the end if heavier); drop-in (no, unless told).
- Look trade-offs: default + show at the end.
