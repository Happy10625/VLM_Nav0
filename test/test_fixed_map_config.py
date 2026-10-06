from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from vlm_nav.fixed_map import validate_localization_mode, validate_map_yaml


ROOT = Path(__file__).resolve().parents[1]


def test_localization_mode_contract_is_strict():
    assert validate_localization_mode("slam") == "slam"
    assert validate_localization_mode("AMCL") == "amcl"
    with pytest.raises(ValueError, match="slam.*amcl"):
        validate_localization_mode("automatic")


def test_map_yaml_resolves_and_reads_relative_image(tmp_path):
    image_path = tmp_path / "map.pgm"
    assert cv2.imwrite(str(image_path), np.zeros((4, 6), dtype=np.uint8))
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(
        "image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )

    resolved_yaml, resolved_image = validate_map_yaml(yaml_path)

    assert resolved_yaml == yaml_path.resolve()
    assert resolved_image == image_path.resolve()


@pytest.mark.parametrize(
    "document,error",
    [
        ("image: missing.pgm\n", "does not exist"),
        ("resolution: 0.05\n", "image"),
        ("not: [valid", "cannot parse"),
    ],
)
def test_invalid_map_fails_explicitly(tmp_path, document, error):
    yaml_path = tmp_path / "bad.yaml"
    yaml_path.write_text(document)
    with pytest.raises(ValueError, match=error):
        validate_map_yaml(yaml_path)


def test_slam_and_amcl_launch_paths_are_mutually_selected_and_forwarded():
    mapping = (ROOT / "launch" / "mapping.launch.py").read_text()
    system = (ROOT / "launch" / "system.launch.py").read_text()
    navigation = (ROOT / "launch" / "navigation.launch.py").read_text()

    assert 'if mode == "slam"' in mapping
    assert 'package="slam_toolbox"' in mapping
    for package in ("nav2_map_server", "nav2_amcl", "nav2_lifecycle_manager"):
        assert f'package="{package}"' in mapping
    for argument in ("localization_mode", "map", "amcl_scan_topic"):
        assert f'DeclareLaunchArgument("{argument}"' in system
        assert f'"{argument}": LaunchConfiguration("{argument}")' in system
    assert '"amcl_overrides_file" if mode == "amcl"' in navigation


def test_ranger_amcl_profile_and_fixed_map_nav2_contract():
    amcl = yaml.safe_load((ROOT / "config" / "ranger_amcl.yaml").read_text())[
        "amcl"
    ]["ros__parameters"]
    nav2 = yaml.safe_load(
        (ROOT / "config" / "nav2_amcl_overrides.yaml").read_text()
    )

    assert amcl["use_sim_time"] is False
    assert amcl["robot_model_type"] == "nav2_amcl::OmniMotionModel"
    assert amcl["laser_model_type"] == "likelihood_field_prob"
    assert amcl["scan_topic"] == "/scan"
    assert amcl["tf_broadcast"] is True
    assert amcl["set_initial_pose"] is False
    assert [amcl[f"alpha{index}"] for index in range(1, 5)] == [0.1] * 4
    assert amcl["alpha5"] == 0.2
    assert amcl["min_particles"] == 500
    assert amcl["max_particles"] == 2000
    assert amcl["max_beams"] == 120

    for costmap_name in ("global_costmap", "local_costmap"):
        params = nav2[costmap_name][costmap_name]["ros__parameters"]
        assert params["use_sim_time"] is False
        assert params["plugins"] == ["static_layer", "inflation_layer"]
        assert params["static_layer"]["map_subscribe_transient_local"] is True
        assert params["footprint"] == (
            "[[0.36, 0.25], [0.36, -0.25], [-0.36, -0.25], [-0.36, 0.25]]"
        )
    assert nav2["controller_server"]["ros__parameters"]["odom_topic"] == (
        "/fastlio/odom"
    )
    assert nav2["velocity_smoother"]["ros__parameters"]["odom_topic"] == (
        "/fastlio/odom"
    )


def test_amcl_mode_disables_frontier_without_removing_slam_default():
    robot = yaml.safe_load((ROOT / "config" / "robot.yaml").read_text())
    system = (ROOT / "launch" / "system.launch.py").read_text()
    navigator = (ROOT / "vlm_nav" / "vlm_navigator.py").read_text()

    assert robot["vlm_nav"]["ros__parameters"]["allow_frontier_after_scan"] is True
    assert "localization_mode" in system
    assert "' != 'amcl'" in system
    assert 'not bool(self.p.allow_frontier_after_scan)' in navigator
    assert 'self.fail_safe("Full scan completed without finding target")' in navigator


def test_existing_validation_and_check_scripts_carry_fixed_map_contract():
    start = (ROOT / "scripts" / "start_navigation_validation.sh").read_text()
    check = (ROOT / "scripts" / "check_system.sh").read_text()

    for argument in ("localization_mode", "map", "amcl_scan_topic"):
        assert f'{argument}:="$' in start
    assert "check_lifecycle_pair /map_server /amcl" in check
    for parameter in (
        "controller_server odom_topic /fastlio/odom",
        "velocity_smoother odom_topic /fastlio/odom",
        "vlm_nav arrival_odom_topic /fastlio/odom",
        "vlm_nav allow_frontier_after_scan False",
    ):
        assert parameter in check
    assert "check_fastlio_odom_health" in check
    assert "FASTLIO_ODOM_MIN_RATE_HZ" in check
    assert "/local_costmap/costmap_raw" in check


def test_fixed_map_wrapper_reuses_existing_startup_flow_and_stays_disarmed():
    wrapper = (ROOT / "scripts" / "start_fixed_map_navigation.sh").read_text()

    assert '"${script_dir}/start_hardware.sh"' in wrapper
    assert '"${script_dir}/start_navigation_validation.sh"' in wrapper
    assert "--localization-mode amcl" in wrapper
    assert '--map "$map_yaml"' in wrapper
    assert '--amcl-scan-topic "$amcl_scan_topic"' in wrapper
    assert 'map_yaml=""' in wrapper
    assert "必须使用 --map MAP.yaml" in wrapper
    assert "lab_ranger_20261006" not in wrapper
    assert "arm.sh" not in wrapper
    assert "enabled=false" in wrapper
