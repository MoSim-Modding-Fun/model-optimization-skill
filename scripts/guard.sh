#!/bin/bash
# Resource guard (bash entry point; the logic is in guard.py, which runs on Linux, Windows/Git Bash and macOS).
#   LOCK=1 MEM=4G CPU=800% guard.sh <command...>      heavy job (pipeline, vis, render pass, batch cleanup)
#   LOCK=0 MEM=2G guard.sh python3 helper.py ...       small helper (still capped and gated)
# Options and behaviour: see guard.py's header (flags --lock/--no-lock --mem --cpu --slots --lock-name also work here).
H="$(cd "$(dirname "$0")" && pwd)"
for c in python3 python py; do
  if command -v "$c" >/dev/null 2>&1 && "$c" -c 'import sys; sys.exit(sys.version_info < (3, 8) or sys.platform in ("cygwin", "msys"))' >/dev/null 2>&1; then
    exec "$c" "$H/guard.py" "$@"
  fi
done
echo "guard.sh: no Python 3.8+ found (python3 / python / py)" >&2; exit 127
