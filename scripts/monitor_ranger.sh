#!/usr/bin/env bash
set -eo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$repo/RangerVLM_env.bash" local
exec /usr/bin/python3 "$repo/scripts/ranger_monitor.py" "$@"
