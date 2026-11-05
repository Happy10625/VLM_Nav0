#!/usr/bin/env bash
# Optional host only; the original entry remains independent of this framework.
set -eo pipefail
ranger_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec /usr/bin/python3 "$ranger_repo/tools/ranger_agent/runtime.py" --bootstrap "$@"
