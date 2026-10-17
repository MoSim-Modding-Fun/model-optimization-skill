# SDS MK5n with the "MK5 Lifted Turret Assembly" (library "MK5n A Lifted" / "MK5n B Lifted", 2026-10-08):
# rmk5n, plus the lifted turret's main piece decimated like rmk5n's turret (shape routing reads it as round and the lathe
# made it a puck). Piece names from mk5n_extract.py of a robot's "SDS MK5n <1>" (A) and "SDS MK5n <3>" (B).
from robots.rmk5n import *  # noqa: F401,F403
from robots import rmk5n as _base

FORCE_METHOD = dict(_base.FORCE_METHOD, **{"MK5 Lifted Turret Assembly.003~000": "decimate",
                                           "MK5 Lifted Turret Assembly.001~000": "decimate"})
