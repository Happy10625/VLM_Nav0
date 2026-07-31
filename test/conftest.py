"""Ensure test discovery imports this checkout instead of an older ROS install."""

from pathlib import Path
import importlib
import sys


ROOT = str(Path(__file__).resolve().parents[1])
if sys.path[0] != ROOT:
    sys.path.insert(0, ROOT)

# ROS/colcon pytest plugins can import the previously installed package before
# pytest adds the checkout root. Remove only that stale package tree so test
# modules resolve the source files under ROOT.
loaded = sys.modules.get("vlm_nav")
loaded_path = str(getattr(loaded, "__file__", "")) if loaded else ""
SOURCE_PACKAGE = str(Path(ROOT) / "vlm_nav")
if loaded is not None and not loaded_path.startswith(SOURCE_PACKAGE + "/"):
    for module_name in list(sys.modules):
        if module_name == "vlm_nav" or module_name.startswith("vlm_nav."):
            del sys.modules[module_name]

# Pin the package search path before ROS test plugins can prefer a colcon
# install tree later in collection. Submodules still import lazily in tests.
importlib.import_module("vlm_nav")
