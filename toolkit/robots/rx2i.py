# WCP Swerve X2i module on its own (library-asset build "WCP Swerve X2i"). The work file comes from
# x2i_extract.py: one module's named pieces under the root empty "X2I" in the MK4I asset frame (tube-mount bottom
# plate's outer corner at the origin, its bottom = frame-rail top at z = 0, module in -x / +y). Motors are left out of
# the asset (owner), so the pipeline keeps them as they are (they only occlude in the visibility pass).
import os, re

ROOT = "X2I"


def part(name):
    return re.sub(r"\.\d{3}$", "", name)


def group_of(o):
    return "main"


LIB_RULES = []
LIMELIGHT_PARTS = set()
LIB_ASSEMBLIES = []
EXPLODE = set()
KEEP_AS_IS = {"Kraken X60 powered by Talon FX (WCP-0940)", "Kraken X44 Brushless Motor"}
REBUILD_KEEP_ALL_HOLES = set()


def is_tube(p, me):
    return False


def is_chain(p, me):
    return False


def is_belt(p):
    return False


SPRINGS = {}
# module frame: x -175..0, y 0..175, z -95 (tread) .. 70 (motor tops)
_C = (-0.088, 0.088, -0.01)
VIEWS = {
    "iso": (_C, (-1.0, -1.25, 0.85), 0.75),
    "iso_back": (_C, (1.1, 1.2, 0.9), 0.75),
    "side": (_C, (-1.0, 0.0, 0.15), 0.7),
    "front": (_C, (0.0, -1.0, 0.15), 0.7),
    "top": (_C, (0.05, -0.08, 1.0), 0.75),
    "under": (_C, (-0.4, -0.5, -0.9), 0.7),
}

# asset-build rules (the MK4I asset's level of detail): round pieces are clean lathe remodels (owner: no grooves),
# bearings one colour (owner), screw threads go (lathes take the major diameter)
PRE_WELD_M = 1e-6
MERGE_DIST = 5e-5
ROUND_BY_SHAPE = True
LATHE_REMODEL = True
SHADE_LATHE = True
LATHE_DISSOLVE_DEG = 1.0
LATHE_MATERIAL_RINGS = lambda p: "bearing" not in p.lower()
REV_KEYS_EXTRA = ("screw", "bolt", "shcs", "bhcs", "phcs", "standoff", "spacer", "locknut", "snapring", "magnet")
FORCE_METHOD = {
    "X2 Molded Wheel": "section lathe",  # keep the recessed hub faces (as the MK5n molded wheel)
    # run 1: the motor-pinion retaining screws (flanged pan heads) passed the hex test and became hex prisms
    "WCP-1740 Rev1": "lathe", '#10-32 x .250" L PHCS (Steel, Black Oxide) (10-Pack) (WCP-1740)': "lathe",
    # (tried for asset v2: "smooth lathe" on the 88t / double / 38t gears gave 21,290 tris vs 21,056 -> not used)
}
FLUSH_BORE_KEEP_STEPS = False
HEX_BODY_FIT = True
HEX_THROUGH_BORE = False
DECIMATE_MAX_TARGET = 1500
DECIMATE_LIGHT_FRAC = 0.7


def PLATE_HOLE_SIDES(r_mm):
    import geo
    return 6 if r_mm < 3 else geo.ngon_segments(r_mm)
