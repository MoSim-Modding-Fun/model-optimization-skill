# TEMPLATE robot config. Copy it to r<name>.py in one of your robot config folders (robot_config_dirs /
# SIMPLIFY_ROBOT_DIRS) and select it with ROBOT=<name>. The names defined below (not commented) are the ones the
# scripts read without a default; every commented line is an optional knob with its pipeline default, so a copy of
# this file runs as it is and each knob is switched on only when the robot needs it.
import re

import numpy as np

# Root object of the imported CAD (verify.py lists every other unparented object). None = no single root.
ROOT = None


def part(name):
    """Object name -> part name: Blender's ".001" / ".002" duplicate suffixes stripped (repeated ones too).
    Add exporter suffixes here when a CAD has them (e.g. "<1>", "~003")."""
    return re.sub(r"(\.\d{3})+$", "", name)


# ---------------------------------------------------------------- visibility groups (vis.py)
def group_of(o):
    """Rigid group an object moves with; "main" = everything static (chassis, bumpers, electronics). A part hidden
    only behind a moving mechanism is not hidden, so give each mechanism that moves its own group, e.g.:
        p = o.parent
        while p:
            if p.name.startswith("Elevator Stage 2"): return "elevator stage 2"
            if p.name.startswith("Intake Arm"): return "intake"
            p = p.parent
    """
    return "main"


# SEE_THROUGH_GROUPS = ()           # groups of clear parts (polycarbonate) that never hide anything or fill a hole
# VIS_DENSE = False                 # hole visibility from the centre + 6 rim points (slots seen only straight-on)
# FLOOR_Z = None                    # floor height in m; None = the lowest object (pin it if helper cubes hang below)
NO_FLOOR_ALL = True                 # owner default: the underside is seen, so the floor hides no hole (False only if
                                    # the owner says the underside is never seen; library asset builds: always True)
# NO_FLOOR_OCCLUSION = ()           # part names whose underside holes count as visible (rails seen from below)
# HEAD_CYL_NO_OCCLUDE = False       # True / part-name set: "head cylinder" parts' CAD shank and tail hide no holes

# ---------------------------------------------------------------- library swaps (pipeline.py)
# (exact part or mesh name, library .blend relative to Components/, asset OBJECT name inside that file), e.g.
#   ("Kraken X60 Brushless Motor", "WCP-0940 (Kraken X60)/WCP-0940 (Kraken X60).blend", "WCP-0940 (Kraken X60)"),
LIB_RULES = []
# multi-piece swaps: dicts with "select" (fn(obj) -> bool), optional "groups" (fn(pieces) -> [[pieces]]), "target":
# "largest" (or "anchor" fn + "object" name), "lib", "asset", "label", optional "flip", "max_mm", "bbox_tiebreak"
LIB_ASSEMBLIES = []
# part names fitted against the Limelight assets; a dict {part: "Limelight 3G"} forces one asset (no flip on this path)
LIMELIGHT_PARTS = set()
# LIB_FLIP = {}                     # {part: "X" / "Z90" / "X+Z90"}: turn the fitted asset about its own axes
# LIB_MAX_MM = {}                   # {part: mm}: fit limit instead of 3 mm (CAD with wires or connectors)
# LIB_NUDGE_MM = {}                 # {mesh or part: (dx, dy, dz)}: world shift in mm after a visual check
# LIB_ABSORB = {}                   # {library part: {other part names}}: pieces the swapped body stands for -> REVIEW
# LIB_COMBOS = {}                   # {part: [(gearbox blend, asset), (motor blend, asset)]}: gearbox + motor, one mesh
# LIB_IDENTITY = {}                 # {part: (blend, asset[, "collection"[, centre]])}: asset built in the part's frame
# LIB_IDENTITY_MOTORS = {}          # {part: [(blend, asset, label, 4x4 mount)]}: library motors next to a module asset
# LIB_EMPTY_IDENTITY = {}           # {empty name prefix: (blend, collection[, "object"[, key regex]])}: assembly assets
# LIB_CAD_COLOURS = ()              # parts whose library swap keeps the CAD's colours (wheels, treads, modules)
LIB_KEEP_HEAVIER = "all"            # library swaps go ahead even when the asset is heavier than the CAD part (owner:
                                    # library by default, A/B at the end); or a set of part names / LIB_ASSEMBLIES labels
KRAKEN_CAP = True                   # library Kraken X60s get a "kraken cap" only where the CAD had one (False: never)
# KRAKEN_ASSET = ("WCP-0940 (Kraken X60)/WCP-0940 (Kraken X60).blend", "WCP-0940 (Kraken X60)")  # motor the cap fits

# ---------------------------------------------------------------- per-part rules (pipeline.py)
KEEP_AS_IS = set()                  # part names already simple enough: left exactly as they are
REBUILD_KEEP_ALL_HOLES = set()      # part names whose rebuild keeps every hole, visible or not
SPRINGS = {}                        # {part: "helix" | "lathe"}: low-poly coil or plain cylinder for CAD springs
EXPLODE = set()                     # merged sub-assembly part names split into loose pieces first (explode.py)
# FORCE_METHOD = {}                 # {mesh or part: method}: "keep", "decimate", "plate", "hex", "sectioned", "lathe",
#                                   #   "lathe-open", "section lathe", "stacked lathe", "smooth lathe", "head cylinder",
#                                   #   "kraken cap", or a BUILDERS key
# BUILDERS = {}                     # {method: fn(part, mesh) -> (mesh | None, info)}: custom builders for FORCE_METHOD
# DECIMATE_RATIO = {}               # {mesh or part: ratio}: fixed Collapse ratio, no quality loop (>= 0.7: none)
# DECIMATE_MAX_TARGET = 1000        # cap on the Collapse tri target per part
# DECIMATE_LIGHT_FRAC = 0.85        # a part whose target is this share of its tris or more stays unchanged
# DECIMATE_KEEP_HOLES = False       # True / "all" / set / dict: decimated parts keep their visible empty holes
PRE_WELD_M = 1e-6                   # weld every mesh first (m): glTF exports can be unwelded polygon soup
# MERGE_DIST = None                 # Merge by Distance (m) before decimation and the final dissolve (<= ~5e-5)
# MESH_PREP = {}                    # {mesh name: fn(me)}: edits of the welded CAD before planning (owner rulings)

# lathes and round parts
LATHE_REMODEL = True                # grooves and notches -> one plain cylinder, profile snapped to clean steps
SHADE_LATHE = True                  # lathe rebuilds get the Smooth by Angle -> WeightedNormal stack
LATHE_MATERIAL_RINGS = lambda p: "bearing" not in p.lower()  # colour rings on lathes; bearings take one colour
FLUSH_BORE_KEEP_STEPS = False       # keep counterbore steps on reopened bores (on, it inflated wheels 2-4x)
# LATHE_TOL_MM = {}                 # {part: mm}: revolve-profile tolerance per part
# LATHE_INSET = {}                  # {part: mm}: pull both end faces in (flush inserts that z-fight)
# LATHE_DISSOLVE_DEG = 5.0          # dissolve angle for lathe / hex rebuilds; 1.0 keeps shallow chamfer facets
# ROUND_EXTRA = ()                  # part names lathed although no round-part keyword matches
# ROUND_BY_SHAPE = False            # nameless round parts go by shape alone (vendor modules with fused bodies)
# REV_KEYS_EXTRA = ()               # extra round-part keywords, matched at word starts (never "nut")
# SPHERES = False                   # balls -> low-poly UV spheres (only parts as deep as they are wide)
# WHEEL_SECTION = ()                # wheels whose fins / windows stay: mid-width section extruded, not a solid disc
# EYEBOLTS = ()                     # eyebolt parts: shank + torus eye

# hex stock
# HEX_BODY_FIT = False              # fit the hexagon to the shaft body, not to E-clips or collars at its ends
# FORCE_HEX_PLAIN = False           # forced "hex" parts become one plain prism, no round journal stubs
# HEX_THROUGH_BORE = False          # keep a hex prism's through-bore when nothing fills it (leave off: hex is solid)

# plates
# PLATE_TOL = {}                    # {mesh or part: mm}: plate outline tolerance (default 0.45)
# PLATE_HOLE_SIDES = None           # fn(r_mm) -> side count for round plate holes


# box tubes: 1x1, 1.5x1, 2x1, 2x2 in stock by its cross-section, measured in the tube's own frame (tubes can sit at an
# angle inside their mesh); add metric sizes to TUBE_SIZES_MM when the robot uses them
TUBE_SIZES_MM = (25.4, 38.1, 50.8)


def is_tube(p, me):
    """Box tube: both short sides match a stock size within 0.6 mm and it is more than 3x as long as it is wide."""
    import build
    d = np.sort(build.frame_dims(me, build.tube_frame(me))) * 1000
    fits = all(min(abs(x - s) for s in TUBE_SIZES_MM) < 0.6 for x in d[:2])
    return fits and d[2] > 3 * d[1]


def is_chain(p, me):
    return "chain" in p.lower()


def is_belt(p):
    return "belt" in p.lower()


# TUBE_MITRED = False               # tubes without a square end face (mitred ends) built as wall slabs
# TUBE_THROUGH_TEST = False         # cut a wall loop only if rays cross the tube (engraved pockets stay solid)
# TUBE_FILL_IRREGULAR = False       # True / set: engraved script or logo loops on tube walls filled, not cut

# ---------------------------------------------------------------- review collections and shading
# EXTRA_DUPLICATES = ()             # objects that duplicate another in place with a different mesh -> REVIEW
# EXTRA_HIDDEN = ()                 # objects the owner wants in REVIEW - Hidden whatever the visibility pass says
# EXTRA_BARELY = ()                 # objects to put in REVIEW - Barely visible although a few pixels were seen
# REVIEW_AFTER = {}                 # {mesh name: reason}: parts the simplification itself hides -> REVIEW
# SHADE_EXTRA = ()                  # part names that get the shading stack whatever their method
# SHADE_SKIP = ()                   # part names that never get it
# DISSOLVE_DEG_PARTS = {}           # {part: deg}: final dissolve angle per part (gently curved sheets)
# NORMALS_PARITY = ()               # non-manifold parts whose normals come from a ray-parity vote, not a recalc

# ---------------------------------------------------------------- kit / module asset builds
# LIB_SUBDIR = ""                   # library folder under Components/ for kit_build.py assets
# ASSETS = {}                       # {key: {"src", "key", "folder", "asset", "opts"}}: kit_build.py asset specs
# KIT_OPTS = {}                     # kit_asset.py options applied to every asset (tags, finish candidates)
# MK5N_SPLIT = {}                   # {"Housing" / "Turret" / "Wheel": CAD part}: mk5n_split.py targets

# ---------------------------------------------------------------- comparison cameras (compare.py)
# {name: (target xyz m, direction from the target, distance m)}; set _C to the middle of the robot's bounding box
_C = (0.0, 0.0, 0.4)
VIEWS = {
    "iso_fl": (_C, (1.0, -1.25, 0.85), 3.5),
    "iso_br": (_C, (-1.1, 1.2, 0.9), 3.5),
    "front": (_C, (0.0, -1.0, 0.15), 3.2),
    "right": (_C, (1.0, 0.0, 0.15), 3.2),
    "top": (_C, (0.05, -0.08, 1.0), 3.5),
    "under": ((0.0, 0.0, 0.1), (0.4, -0.5, -0.9), 2.8),
}
