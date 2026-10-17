# REV 6" ION omni wheel (fused vendor body split by split_extract.py into "Omni 0" = hub, "Omni 1.NNN" = 16 rollers),
# library-asset build. Rollers -> solid lathe barrels; hub -> plate rebuild or decimate.
import re

ROOT = None


def part(name):
    return re.sub(r"\.\d{3}$", "", name)


def group_of(o):
    return "main"


LIB_RULES = []
LIMELIGHT_PARTS = set()
LIB_ASSEMBLIES = []
EXPLODE = set()
KEEP_AS_IS = set()
REBUILD_KEEP_ALL_HOLES = set()


def is_tube(p, me):
    return False


def is_chain(p, me):
    return False


def is_belt(p):
    return False


SPRINGS = {}
VIEWS = {"iso": ((0, 0, 0), (1.0, -1.25, 0.85), 0.5)}
PRE_WELD_M = 1e-6
MERGE_DIST = 5e-5
LATHE_REMODEL = True
SHADE_LATHE = True
LATHE_DISSOLVE_DEG = 1.0
FORCE_METHOD = {"Omni 1": "lathe", "Omni 0": "decimate"}  # hub: the plate rebuild capped the spoked face
DECIMATE_MAX_TARGET = 1500
DECIMATE_LIGHT_FRAC = 0.7
NO_FLOOR_ALL = True
