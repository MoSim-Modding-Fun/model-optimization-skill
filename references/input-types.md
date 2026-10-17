# Input types

What changes in the workflow depending on what you were handed. The inventory tells you most of it; confirm the rest
at intake. `$TK` below is the skill's `toolkit/` folder.

## Contents
1. Owner-imported `.blend` from an Onshape / SolidWorks glTF
2. `.glb` / `.gltf` you import yourself
3. STEP
4. The game's own FBX (normal flow, the default)
5. The game's own FBX as a drop-in (only when told)
6. Models built -Y up
7. Very large CADs: batched cleanup first
8. Merged sub-assemblies and fused vendor parts
9. Hand-made block-outs

## 1. Owner-imported `.blend` from an Onshape / SolidWorks glTF (the common case)

- **Unwelded polygon soup.** Many glTF meshes have every triangle on its own three vertices. Anything that walks edges
  (tube walls, plate loops, hole plugging, island splitting) silently fails. Weld every mesh at 1 µm before analysis
  (`PRE_WELD_M = 1e-6`); custom normals survive.
- **Custom normals** are on every mesh and often broken: flat plates look crumpled. The QA "before" renders strip them.
  Never judge CAD shading from raw glTF normals.
- **Instancing.** Onshape exports share mesh datablocks between copies (work per datablock: unique tris drive the run
  time, not placed tris). SolidWorks exports give every object its own mesh: run `explode.py` first, which makes
  identical meshes share one datablock (one robot went 1,117 → 261 meshes).
- **Names.** Onshape: part names with Blender `.NNN` suffixes, `meshNNNN_mesh` datablocks, assembly empties at the
  world origin with every piece baked in world space (so module or asset frames must come from a key piece, not an
  empty). Generic names ("Part 40") cover unrelated parts in different sub-assemblies: key every override on the mesh
  datablock name. SolidWorks: `Part-<n>` style names and `defaultplastic.<n>` materials whose colour lives only in the
  Principled BSDF; treat those colours as uninformative.
- **Helpers.** Onshape "Origin / Reference Cube" markers hang 25–110 mm below the wheels and drop the floor; every
  underside hole then reads as visible. Set `FLOOR_Z` to the wheel contact plane and move the cubes to a REVIEW
  collection.
- **The owner's own simplified parts** carry Decimate → Smooth by Angle → WeightedNormal; keep them.
- **Draco-compressed glTF** decodes fine in Blender's importer; check once that flat faces are exactly planar before
  worrying about quantisation (they were).

## 2. `.glb` / `.gltf` you import yourself

Only when asked ("Import it into a .blend") or for a small vendor file handed to you. Import with the default glTF
importer from an empty scene (`bpy.ops.wm.read_factory_settings(use_empty=True)` first), through `bl.sh` with a raised
`MEM` (a 250 MB glb needed ~6 GB; say so), save compressed, then check the up axis and the floor. ~30–60 s per 100 MB.
Set `save_version = 0` or check the output before rerunning, to avoid stray `.blend1` files.

## 3. STEP

Blender has no STEP importer. `$TK/step_import.py` runs in a venv with OpenCASCADE bindings, `$TK/step_build.py` builds
the `.blend` the pipeline expects. Ask before installing (it downloads ~1.3 GB).

```bash
V="<robot>/_work/occ_venv"; python3 -m venv "$V" && "$V/bin/pip" install cadquery-ocp
LOCK=0 MEM=6G guard.sh "$V/bin/python" $TK/step_import.py export "<robot>.step" "<robot>/_work/import/e050" 0.05 0.25
MEM=6G bl.sh --factory-startup --python $TK/step_build.py -- "<robot>/_work/import/e050" "<robot>.blend"
```

- Tolerances 0.05 mm linear / 0.25 rad angular gave ≥ 25 segments on any hole; meshing takes seconds, so try 2–3
  settings on a denser robot. A 458 MB STEP: read + transfer ~1 min, meshing 20 s, build ~1 min.
- Mesh names are deterministic (`m<part>s<solid>[c<style>]`), so config overrides can key on them. Identical geometry
  under other product names (copies "(1)", "v2", "(Mirror)") shares one mesh; write `LIB_RULES` for every product name.
- Multi-body parts become an empty with one child per solid. Unnamed products (`COMPOUND`, `SOLID`, `Body12`) take
  their assembly's name.
- STEP colour names lie ("ABS (White)" on a red colour): materials are named by the sRGB value.
- Check for bodies missing from the export: compare each part's size with its name (a "3.000in" flex wheel measured
  38 mm: only the fluted hub was exported). Ask.
- OCP 8 quirks: `TDF_LabelSequence` is `OCP.collections.Sequence_TDF_Label`; an XCAF document saved as XBF reopened
  empty (re-read the STEP); `node.LocalTrsf` needs `.Transformation()`.

## 4. The game's own FBX (normal flow, the default)

"Unless specifically told, assume it is not being replaced directly in unity." Treat the game's FBX like any other
source: import it with full names (`$TK/fbx_import.py`; the stock importer cuts names over 63 bytes to 55 chars + a
hash), simplify, deliver `<name> - simplified.blend`. New objects, library colours and dropped material slots are all
fine. Modifier stacks don't survive an FBX, so "keep the owner's simplified parts" can't be detected: ask if belts or
chains already look like ~260-tri loops.

## 5. The game's own FBX as a drop-in (only when told)

When the owner says the result will overwrite the game's FBX under an unpacked prefab (MeshFilters reference the FBX's
meshes by fileID), the contract is strict:

- **Keep**: every object name, mesh datablock name, mesh sharing, each mesh's material slot list (count and order, and a
  face in every slot: Unity drops empty submeshes and every later renderer material shifts), and node order. Unity names
  a shared mesh after the FIRST node of its own hierarchy walk and derives the fileID from that name; Blender's
  exporter sorts objects by name, so a plain export can rename shared meshes. Nothing may be added that the prefab
  should see (new objects never render).
- **Flow**:
  1. Full-name import → `game_restore.py dump` → read the prefab with `git show` (MeshFilters → meshes, Rigidbody +
     joints → moving groups, renderer materials → panels clear only in game, objects the prefab never draws → a
     see-through `"notgame"` group).
  2. **Round trip of the untouched import first**: `fbx_dropin.py export` with `FBX_OPTS` (units: a game FBX may have
     UnitScaleFactor 100) and `FBX_ORDER=<game fbx>` → `fbx_dropin.py compare` clean (`first_model_diffs` 0) → the
     **Unity probe** (`$TK/unity_probe/`: a throwaway Unity project imports the FBX with the game's `.meta` and checks
     every prefab mesh reference) with 0 problems. Blender-side compares once passed while 19 prefab mesh references
     were broken in Unity.
  3. Every run: pipeline → implode → phase3 → `game_restore.py restore` (library materials onto the CAD's slots by
     CIELAB nearest colour; per-face mapping painted stripes) → export → compare → probe. Check `restore.json` split
     meshes and face counts after every restore (an exploded duplicate once emptied its visible twin).
  4. Deliver the `.blend` (live shading stack) AND the drop-in FBX (shading baked once per shared mesh; Blender's
     export with Apply Modifiers gives every modified object its own renamed mesh), plus an export-ready baked
     `.blend` if the owner exports themselves. Report "drawn in game" tris separately: REVIEW objects still render.
  5. After the owner deletes REVIEW objects: back up, diff, `game_remake.py` (deleted objects come back as placeholder
     meshes when no kept object shares their mesh), put back deleted FIRST users of still-shared meshes in the export
     source, re-export, compare, probe. Never export the owner's edited file directly.
- `verify.py` exports with default settings: a smoke test only, not proof for a drop-in.

## 6. Models built -Y up

Some Onshape robots come in standing on their side (wheels at y = 0). The pipeline needs Z up (floor = min z, compare
views): run everything on a work copy where ONLY the root empty is rotated -90° about X, restore the root's exact
rotation (mode and quaternion) in the deliverable, and compare every object's world matrix with the ORIGINAL import
(`$TK/tfcheck.py`, 0.0 deviation). Objects the run adds must sit under the root. `VIEWS` and `LIB_NUDGE_MM` are in the
work frame. Ask which orientation to deliver.

## 7. Very large CADs: batched cleanup first

Several million unique tris, or single meshes over ~1M: run Phase 1 before the pipeline.

```bash
LOCK=1 MEM=6G guard.sh python3 $TK/cleanup/run_phase1.py "<dir>/<robot>.blend" [--cap 1000000] [--alone 500000]
```

- `cleanup/plan.py` hashes local geometry + materials into unique units and packs them into batches of ≤ 1M unique
  tris (a unit over 500k gets a batch alone). Each batch runs in its own Blender process on the latest checkpoint:
  Merge by Distance at 0.01 mm (never above ~0.05 mm: thin parts collapse), Limited Dissolve ≤ 5° delimiting materials,
  no boundary dissolve, custom normals cleared; compressed checkpoint + `progress.json` (resumable); renders of the 5
  largest meshes; world bboxes unchanged.
- Timings: 19M tris in ~3 min with RAM free; with RAM starved, one batch went from 7 s to 160 s. Merge by Distance is
  the slow step (a 2M-tri swerve module alone took ~650 s). Peak 2–4 GB per batch.
- Deliver `<name> - cleaned.blend` too, and report original → cleaned → simplified.
- Uncompressed GB-size checkpoints written back to back pushed a RAM-starved machine into I/O wait: compress them.
- A runner that deleted its own old checkpoints was blocked by the safety check: keep them, clean up at the end.

## 8. Merged sub-assemblies and fused vendor parts

- **One mesh holding hundreds of islands and dozens of materials is an assembly** (a whole swerve module, a gearbox +
  motor). `EXPLODE` it into `<object>~NNN` pieces before the pipeline and `implode.py` them back after (name, parent,
  transform kept). Library fits on assemblies need the pieces.
- **Vendor solids fused into one body** (a swerve turret or housing): split by connected islands only. Splitting by
  colour gives open shells that the lathe closes into pucks. What stays fused gets a Collapse chosen by render error.
- **Gearbox + motor in one mesh**: `LIB_COMBOS` fits the motor to the best of the 6 largest islands and the gearbox to
  the rest.
- **Motors exported as many loose pieces** (33 for one motor): group them by touching bounding boxes (union-find), not
  by name, then fit as one point cloud (`LIB_ASSEMBLIES`).

## 9. Hand-made block-outs

A fused, single-sided, unwelded block-out mesh per module: raw islands are half-cylinders and C-strips, and a global
weld fuses neighbours into blobs. What works: raw islands; islands whose extents are all < 25 mm and whose bboxes touch
(within 0.02 mm) clustered into one part by union-find; each part welded on its own; per-part pipeline with builders for
flat single-sided sheets, open hole walls and open revolves (anything open that isn't a clean revolve stays as it is:
the lathe closes open pockets into plugs); joined back into the same mesh. Render QA with `CULL=both`.
