#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"

# Keep unrelated user/system pytest plugins out of this deterministic test run.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="${project_dir}:${PYTHONPATH:-}"

exec python3 -m pytest -q test
