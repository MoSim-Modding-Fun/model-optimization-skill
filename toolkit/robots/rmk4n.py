# SDS MK4n swerve module on its own (library-asset build "MK4n A" / "MK4n B", vendor CAD from the owner).
# Work files come from mk4n_extract (_work/mk4n): every part is a named mesh object under the root empty
# "MK4N A" / "MK4N B", in the MK4i asset's frame (L-plate outer corner at the origin, base plate on z = 0, module in
# -x / +y). Motors are left out of the asset (owner), so the pipeline keeps them as they are.
import os, re

ROOT = os.environ.get("MK4N_ROOT", "MK4N A")


def part(name):
    return re.sub(r"\.\d{3}$", "", name)


def group_of(o):
    return "main"


LIB_RULES = []
LIMELIGHT_PARTS = set()
LIB_ASSEMBLIES = []
EXPLODE = set()
KEEP_AS_IS = {"Kraken X60 Brushless Motor"}
REBUILD_KEEP_ALL_HOLES = set()


def is_tube(p, me):
    return False


def is_chain(p, me):
    return False


def is_belt(p):
    return p == "MK4n Belt"


SPRINGS = {}
# module frame: A spans x -170..0, y 0..232; B spans x -232..0, y 0..170; z -91 (tread) .. 81 (top screws)
_C = (-0.1, 0.1, 0.0)
VIEWS = {
    "iso": (_C, (-1.0, -1.25, 0.85), 0.8),
    "iso_back": (_C, (1.1, 1.2, 0.9), 0.8),
    "side": (_C, (-1.0, 0.0, 0.15), 0.75),
    "front": (_C, (0.0, -1.0, 0.15), 0.75),
    "top": (_C, (0.05, -0.08, 1.0), 0.8),
    "under": (_C, (-0.4, -0.5, -0.9), 0.75),
}

# asset-build rules (the MK4i asset's level of detail): round pieces are lathes (no screw threads), the wheel tread is
# a plain band (no tread pattern)
PRE_WELD_M = 1e-6  # glTF export: weld polygon soup before analysis
ROUND_BY_SHAPE = True
SPHERES = True
TREAD = "Tread, Black Nitrile 4in X 1.5in"
LATHE_TOL_MM = {TREAD: 4.0}
