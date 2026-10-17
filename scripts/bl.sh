#!/bin/bash
# Headless Blender through the guard: the configured Blender (config "blender", SIMPLIFY_BLENDER, PATH or the usual
# install folders) with -b -t ${THREADS:-8}; heavy-job lock and a 4 GB cap by default.
#   bl.sh <blender args...>                    e.g. bl.sh robot.blend --python vis.py -- vis.json 80 1400
#   LOCK=0 MEM=2G bl.sh ...                    short checks (inventory, one render) still get the RAM gate + cap
#   MEM=12G THREADS=16 bl.sh ...               big CADs (say why in the log; check free memory first)
# Never call blender directly, not even for a 2-second --python-expr check.
H="$(cd "$(dirname "$0")" && pwd)"
for c in python3 python py; do
  if command -v "$c" >/dev/null 2>&1 && "$c" -c 'import sys; sys.exit(sys.version_info < (3, 8) or sys.platform in ("cygwin", "msys"))' >/dev/null 2>&1; then
    exec "$c" "$H/guard.py" --blender "$@"
  fi
done
echo "bl.sh: no Python 3.8+ found (python3 / python / py)" >&2; exit 127
