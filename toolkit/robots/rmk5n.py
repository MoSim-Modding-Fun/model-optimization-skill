# SDS MK5n swerve module on its own (library-asset build "MK5n A" / "MK5n B"). Work files come from
# mk5n_extract.py: the module's pieces under the root empty "MK5N A" / "MK5N B" in the MK4I asset frame (L-plate outer
# corner at the origin, z = 0 at the housing's base-plate bottom = frame-rail top, module in -x / +y). The vendor's fused
# turret and housing are split into "<part>~NNN" pieces by island and colour. Motors are left out of the asset (owner),
# so the pipeline keeps them as they are (they only occlude in the visibility pass).
import os, re

ROOT = os.environ.get("MK5N_ROOT", "MK5N A")


def part(name):
    return re.sub(r"\.\d{3}$", "", name)


def group_of(o):
    return "main"


LIB_RULES = []
LIMELIGHT_PARTS = set()
LIB_ASSEMBLIES = []
EXPLODE = set()
KEEP_AS_IS = {"Kraken X60 Brushless Motor", "Kraken X44 Brushless Motor"}
REBUILD_KEEP_ALL_HOLES = set()


def is_tube(p, me):
    return False


def is_chain(p, me):
    return False


def is_belt(p):
    return False


SPRINGS = {}
# module frame: about x -160..0, y 0..190, z -95 (tread) .. 59 (housing top)
_C = (-0.08, 0.08, -0.02)
VIEWS = {
    "iso": (_C, (-1.0, -1.25, 0.85), 0.75),
    "iso_back": (_C, (1.1, 1.2, 0.9), 0.75),
    "side": (_C, (-1.0, 0.0, 0.15), 0.7),
    "front": (_C, (0.0, -1.0, 0.15), 0.7),
    "top": (_C, (0.05, -0.08, 1.0), 0.75),
    "under": (_C, (-0.4, -0.5, -0.9), 0.7),
}

# asset-build rules (the MK4I asset's level of detail): round pieces are clean lathe remodels (owner: no grooves),
# nameless pieces of the fused bodies go by shape
PRE_WELD_M = 1e-6
MERGE_DIST = 5e-5
ROUND_BY_SHAPE = True
LATHE_REMODEL = True
LATHE_DISSOLVE_DEG = 1.0
FORCE_METHOD = {"SDS MK5 Molded Wheel": "section lathe"}  # keep the wheel's recessed hub face
FLUSH_BORE_KEEP_STEPS = True
DECIMATE_MAX_TARGET = 1500
DECIMATE_LIGHT_FRAC = 0.7


def PLATE_HOLE_SIDES(r_mm):
    import geo
    return 6 if r_mm < 3 else geo.ngon_segments(r_mm)
