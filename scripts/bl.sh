#!/bin/bash
# Headless Blender through guard.sh. Default = heavy job: machine-wide lock, 4 GB cap, 8 threads.
#   bl.sh <blender args...>                    e.g. bl.sh robot.blend --python vis.py -- vis.json 80 1400
#   LOCK=0 MEM=2G bl.sh ...                    short checks (inventory, one render) still get the RAM gate + scope
#   MEM=12G THREADS=16 bl.sh ...               big CADs (say why in the log; check `free -m` first)
# Never call `blender` directly, not even for a 2-second --python-expr check.
H="$(cd "$(dirname "$0")" && pwd)"
MEM=${MEM:-4G} CPU=${CPU:-800%} LOCK=${LOCK:-1} exec "$H/guard.sh" blender -b -t "${THREADS:-8}" "$@"
