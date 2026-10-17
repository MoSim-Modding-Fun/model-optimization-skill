# Owner rulings (the house style)

Everything the owner (the library maintainer) has ruled across past robot jobs, in their words where it matters. The
words are quoted because the owner answers tersely and the exact wording has been misread before. When someone else
runs the skill, these are the house style: follow them unless that person says otherwise.

If two rulings seem to conflict, the later one wins; say so to the owner rather than picking silently.

## Contents
1. How to work with the owner
2. Asking, deciding, delivering
3. Geometry and look
4. Library assets
5. Decision defaults (how the owner has chosen before)
6. Resources

## 1. How to work with the owner

- **Do exactly what was asked.** "Every X" means every X. "Make a model for the cap" meant one model, not "a cap on
  every motor": a session widened that once, the default spread to other robots, and every extra cap had to be removed
  ("the cap doesnt go on every x60. its only if there was one to begin with."). Don't narrow either.
- **Push back** plainly when a request looks wrong, before doing it.
- **Terse answers.** Decision answers are a few words ("all a is good", "fins", "plugged", "cup", "yeo"). Map each to
  the option you listed and restate the mapping in one line in the same message that continues the work ("D1 → A, the
  library cap"), so a misread is caught in one reply. If two readings fit, ask one short question naming the options.
  A misread answer once cost two rework passes; keep a copy of the pre-change file so a wrong reading reverts in minutes.
- **The owner usually picks the option marked "(Recommended)"**, so the recommendation carries real weight. They
  override it when the image shows a look they care about.
- **The owner edits deliverables by hand** (deletes REVIEW objects or plates, moves parts, adds objects). Before
  touching a delivered file: back it up, diff it against your last delivery (object names, transforms, evaluated tris,
  materials), ask about differences you can't explain, and never restore what they removed.
- **Show numbers.** A look question without tri costs gets "how many faces are your cylinders" back.
- **The owner reviews images while you work** and may answer with annotated screenshots. Fix and send a before/after.
- "Anything of note, just tell me both when you find it and at the end."
- No "Co-Authored-By: Claude …" trailer on any commit, in any repo.

## 2. Asking, deciding, delivering

| Rule | Owner's words / ruling |
|---|---|
| Assume, don't ask | "This should not be asked it should be assumed" (2026-10-16, about swerve modules, kept features, budget and delivery, after the same for moving/clear parts, the underside and imports). Everything the files, measurements or house rules settle is assumed and stated in the plan; ask only for permissions and real blockers. Look trade-offs get the documented default and go into one batch at the end: "Show me anything I have to decide at the end with tri counts and images of both options." |
| Show the plan and keep going | After the inventory, show the plan, the biggest offenders and an "Assumed" list, then run. Don't wait for approval of the plan or the assumptions. A session that paused there was told "when the fuck did i tell you to stop". |
| Mid-run decision windows | When the owner says they have time ("If you have any decisions I need to make now, let me know now"), show the pending decisions then, as AskUserQuestion A/B options with images. Never stop on your own to ask. |
| Not a drop-in | "Unless specifically told, assume it is not being replaced directly in unity." |
| Never modify the input | Outputs next to it: `<name> - simplified.blend`. "If I've already edited a previous output, work on top of my saved file (back it up first) instead of regenerating from scratch." |
| Moving and clear parts | "assume the moving parts and clear parts" (2026-10-16): infer them from the hierarchy, names and materials; don't ask. State the assumptions in the plan. |
| Underside | "assume underside will be seen." (2026-10-16): holes and parts on the underside count as visible (`NO_FLOOR_ALL = True`). |
| Never delete parts | "just dont delete hidden ones, mark them for manual review by me." Hidden, duplicate, barely visible and replaced parts go into `REVIEW - …` collections. The owner may delete REVIEW later; that's their call. |
| Imports | "If all thats provided is not a blend, assume you import" (2026-10-16): when no `.blend` is given, import the `.glb` / `.gltf` / STEP / FBX yourself. (When the owner says they will import, wait.) |
| Decision images | Put them in the review folder AND send them when asking (small JPGs). "Where are the images for the decisions?" came from a session that only described them. |
| Per-object images | "When fully finished I want images of each unique object. at least 2 images, make sure i can see all the detail. it should be separate, one object per image. each a unique id." |
| End marker | Every finished job ends with the line `SIMPLIFICATION DONE`. |
| Unity reminder | "Remind me that the Unity import setting Normals must be 'Import'." |
| Clean up | "Once you respond, remove stuff you used from my RAM to free up space": clear your scratchpad (it lives in RAM) at the end. |
| Pushes | Commit/push only with the owner's OK. They have asked for robots to be pushed "individually … like my previous commits"; prepare the commands if the harness blocks you. |

## 3. Geometry and look

| Topic | Ruling |
|---|---|
| Budget | "should always be min tris" (2026-10-16): the minimum tris with no visible loss; never ask for a target. (Earlier: "Past models have gotten anywhere from 100k to 500k. Aim for the lower end but its not a hard cutoff.") |
| Look bar | Curved parts "should be smooth to the eye from like a few feet away". Big curved parts: Collapse ratio by render-error knee, constant-section rings as true revolves with baked normals. Tri count is secondary to that bar. |
| Round parts | "gears should be remodelled into cylinders". Rollers, gears, pulleys, sprockets, spacers, bearings, bolts, rivets → lathe cylinders; teeth and spoke windows go. Side counts 8 / 16 / 24 / 32 / 48 / 64 for radius < 6 / 6–12 / 12–20 / 20–35 / 35–45 / ≥ 45 mm. |
| Lathes are remodels | "stuff like these should be remodeled not decimated", "you dont need grooves. one cylinder can replace all the grooves", "dont need the bands". Flanges, hubs, bores and cones stay. |
| Bearings | "for all bearings you can use one color". Multi-ring bearings (tapered rollers): "too many tris … make it simple": plain rings, no cage detail. |
| Hex | "any hex shaft should be a 6 sided cylinder." Hex stock and adapters are solid prisms: "hex should never have through bore on hex shafts"; blind holes in a hex piece's stub are filled. Grooved hex stock is a plain hex prism (don't offer the fluted version again). |
| Box tubes | "i need the holes and tube visible": procedural tubes with 6-sided through-holes; holes that are covered or provably invisible may go, backed by a highlight render. Owner-named truss/side plates keep every hole ("On these 2 plates, all holes should remain."). |
| Equal cost | "if theyre equal then do the rebuilding": at about the same tri cost, prefer a clean procedural rebuild over decimation. |
| Chain | "chains should be one loop instead of links" (replaces the older per-link rule still found in old prompt templates). |
| Rivets | Head-only cylinder ("make rivets just one cylinder for the rivet head") only when the robot's rivets total more than 10k tris. |
| Decorative and identity features | "The swirl on the spindexer is a must keep": a rule never flattens a look-defining feature (swirls, truss pockets, engraved team numbers, lettering) without asking. Team numbers stay readable. |
| Decals | Vendor logos/decals become plain shapes ("you dont need decals on the swerve covers. you can just have it be a circle"). |
| No phasing | "i don't want stuff phasing on the side … it looks bad": fitted pieces must not poke through a neighbouring face, even if the CAD does; cut them 0.25 mm inside. |
| Swerve wheels | "Swerve wheel orientation does not matter." Fit library modules on the housing; ignore the steer angle. |
| Ground contact | Keep the CAD as it is. Measure and report the wheels' lowest point before changing anything; a "fix" requested from a misunderstanding had to be undone. |
| The owner's own simplified parts | Parts carrying Decimate → Smooth by Angle → WeightedNormal stay, unless named. The original wording ("look at the ones with modifiers to see how") could also mean "copy my method": ask if a robot has many. |

## 4. Library assets

| Topic | Ruling |
|---|---|
| Library by default | Use the library asset. If it is heavier than the simplified CAD part, keep the library one and show both at the end with tri counts (owner choice, 2026-10-16). The owner has picked heavier library assets when shown both. |
| Colours | Motors and electronics keep the library's colours. Wheels, treads and swerve modules take the robot CAD's colours, **unless the CAD colours are uninformative** (no colour, an export's default colours such as SolidWorks appearances, or the whole module one colour): then keep the library colours. "treads are whatever color it was in cad as long as it uses the asset swerve." |
| Compliant / flex wheels | Always a library wheel of the same OD and width ("make sure any Flex/Compliant wheels use either a model in the asset library or you make one for that size and od and add it to the asset library"), and it must look like the CAD wheel (a no-context subagent judges CLOSE ENOUGH). They keep fins and windows ("so they actually look compliant"). New compliant wheels follow the library's existing compliant-wheel style with "slightly more edges to be more circular" (16 sides). |
| Module assets | "the motors should not be in the swerve asset. this goes for any module you simplify into an asset." "it shouldnt include anything on the inside because the interior is invisible due to the shell." Detail level of the reference module: "it's not everything nor everything visible. just the noticeable things"; "screws should have no treads". |
| Library module on a robot | "if its less tris and looks the same then use the modules in library. If they are more tris lmk." |
| Module revisions | A robot whose module is a different revision than the vendor's gets its own module asset kept with the robot, not in the library ("the … asset shouldnt go in the library"). Another revision of a vendor module that several robots use gets its own library file. |
| Electronics variants | A second model of a part (another signal light) gets its own asset "same fidelity as the one in the library"; don't accept a wrong-size match. A library asset may be replaced by a simpler remodel of the robot's own part when the owner says so. |
| Electronics without a match | "library eth switch and encoder. make sure they're similar poly and style to the existing ones" (2026-10-17): an Ethernet switch and an encoder with no library match became library assets at the poly count and in the style of the existing library electronics (clean primitives, label regions, port recesses, simplified mounts). Default for electronics with no match: a library asset, not a per-robot decimation. |
| Motors and batteries as assets | "Remodel these to be like the kraken and similar. you can omit the port." |
| Cameras | One object, shell and lens, only what is visible from outside, no gaps, no internals ("you only need the part of the lens thats visible--and make sure theres no gaps"); a closed back when something always sits behind it ("just make the back solid"). Limelights: "the box, the lens, and the lights visible on the front, and any details visible on the front." |
| Motor caps | The connector cap goes only on motors whose CAD had one; "align it properly and not off-angle. the flat side should fit". Build it from the library motor's own geometry. |
| Fixing the library | "if none of my agents created the [asset] then just fix it": broken assets nobody made may be fixed, after a backup outside the library folder. |
| Duplicates | Two builds of the same wheel: "just have both". |
| Library git | Owner: ask before committing or pushing. Anyone else: don't touch the library's git. |

## 5. Decision defaults (how the owner has chosen before)

Use these as the default and the "(Recommended)" option when the same kind of choice comes up; still show it at the end
if it changes the look.
- The cheaper option when the difference only shows close up (housing decimation ratio, bearing rings, hex bores, a
  look-matching wheel vs a cheaper similar one).
- Readable team numbers and look-defining features kept, even at a cost (engraved numbers, decorative fins and
  swirls, truss pockets that are clearly deep).
- Visible colour contrast kept ("i want the visible bearings at the top cuz the colors r noticable").
- Library modules when they are lighter and look the same; library electronics even when a bit heavier.
- Solid blocks inside tubes as plain boxes; small holes in solid bars plugged consistently.
- Small holes left open in already-light sheet metal; decals and team numbers kept sharp.
- REVIEW collections kept for the owner's own review.

## 6. Resources

- "Make sure ram and cpu usage doesnt cause anything to crash"; "total ram usage shouldnt exceed like 80 or 90%";
  "Make sure ram check is left and to ensure there is always enough free ram."
- After an incident where the whole Claude app was killed for memory: "What the fuck did I tell you about keeping ram
  usage low." No exceptions for tiny scripts: every process goes through the guard.
- "For directions, ask archived sessions then archive them again when done": one at a time, after a RAM check.
