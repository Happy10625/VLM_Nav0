#!/usr/bin/env bash
# Read-only cooperative Ranger acceptance, with concise Chinese console output.
set -eo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$repo/RangerVLM_env.bash" local
exec /usr/bin/python3 "$repo/scripts/ranger_check.py" "$@"
