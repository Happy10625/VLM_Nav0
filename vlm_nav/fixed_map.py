"""Validation helpers for fixed-map localization launch files."""

from pathlib import Path

import cv2
import yaml


def validate_localization_mode(mode):
    value = str(mode).strip().lower()
    if value not in {"slam", "amcl"}:
        raise ValueError(
            f"localization_mode must be 'slam' or 'amcl', got {mode!r}"
        )
    return value


def validate_map_yaml(yaml_path):
    """Return resolved YAML/image paths after validating a Nav2 map asset."""
    if not str(yaml_path).strip():
        raise ValueError("AMCL localization requires map:=/absolute/path/to/map.yaml")

    map_yaml = Path(yaml_path).expanduser().resolve()
    if not map_yaml.is_file():
        raise ValueError(f"map YAML does not exist or is not a file: {map_yaml}")
    if not map_yaml.stat().st_size:
        raise ValueError(f"map YAML is empty: {map_yaml}")

    try:
        document = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"cannot parse map YAML {map_yaml}: {error}") from error
    if not isinstance(document, dict):
        raise ValueError(f"map YAML must contain a mapping: {map_yaml}")

    image_value = document.get("image")
    if not isinstance(image_value, str) or not image_value.strip():
        raise ValueError(f"map YAML has no usable 'image' path: {map_yaml}")
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    image_path = image_path.resolve()
    if not image_path.is_file():
        raise ValueError(
            f"map image referenced by {map_yaml} does not exist: {image_path}"
        )

    image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if image is None or image.size == 0:
        raise ValueError(f"map image is not readable: {image_path}")
    return map_yaml, image_path
