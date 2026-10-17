# 2" vectored (mecanum-style) intake wheel, right + left, on their own (library-asset build). Work file from
# _work/mec/extract.py: one "Right Mecanum <1>" and one "Left Mecanum <1>" (hub + 6 rollers each) in their hub's
# mesh frame (the left one shifted +120 mm in x). Rollers -> lathe barrels (owner: rollers become lathe cylinders),
# hubs -> decimate with the quality loop (the roller cups and hooks are the look).
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
VIEWS = {"iso": ((0.035, 0, 0), (1.0, -1.25, 0.85), 0.25)}
PRE_WELD_M = 1e-6
MERGE_DIST = 5e-5
LATHE_REMODEL = True
SHADE_LATHE = True
LATHE_DISSOLVE_DEG = 1.0
FORCE_METHOD = {"Right Hub": "decimate", "Left Hub": "decimate", "Right Roller": "lathe", "Left Roller": "lathe"}
DECIMATE_MAX_TARGET = 1500
DECIMATE_LIGHT_FRAC = 0.7
NO_FLOOR_ALL = True
