#!/bin/bash
# Install the skill (Linux, macOS, Git Bash on Windows): the installer itself is install.py, see its header.
#   ./install.sh [--library <clone>] [--robots <folder>] [--robots-root <folder>] [--blender <exe>] [--maintainer]
#                [--clone-library] [--copy] [--yes]
H="$(cd "$(dirname "$0")" && pwd)"
for c in python3 python py; do
  if command -v "$c" >/dev/null 2>&1 && "$c" -c 'import sys; sys.exit(sys.version_info < (3, 8) or sys.platform in ("cygwin", "msys"))' >/dev/null 2>&1; then
    exec "$c" "$H/install.py" "$@"
  fi
done
echo "install.sh: no Python 3.8+ found (python3 / python / py)" >&2; exit 127
