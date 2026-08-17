from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
OBSTACLE_TOPIC = "/vlm_nav/obstacle_cloud"


def test_main_launch_starts_only_the_standard_cloud_filter():
    source = (ROOT / "launch" / "vlm_navigation.launch.py").read_text()

    assert 'executable="obstacle_cloud_filter"' in source
    assert "sparse_obstacle_filter" not in source
    assert "sparse_costmap_filter" not in source


def test_cloud_filter_uses_sensor_qos_and_exact_stamp_message_filter():
    source = (ROOT / "src" / "obstacle_cloud_filter.cpp").read_text()

    assert "rmw_qos_profile_sensor_data" in source
    assert "setTolerance(rclcpp::Duration::from_seconds(0.0))" in source
    assert "setTolerance(rclcpp::Duration::from_seconds(0.1))" not in source
    assert "tf2::durationFromSec(0.0)" not in source
    assert "TimePointZero" not in source
    assert "registerFailureCallback" not in source


def test_diagnostics_are_periodic_and_cloud_callback_only_updates_statistics():
    source = (ROOT / "src" / "obstacle_cloud_filter.cpp").read_text()
    robot = yaml.safe_load((ROOT / "config" / "robot.yaml").read_text())
    callback = source.split("void on_cloud", 1)[1].split(
        "void produce_diagnostics", 1
    )[0]

    assert "diagnostic_updater::Updater" in source
    assert robot["obstacle_cloud_filter"]["ros__parameters"][
        "diagnostic_updater.period"
    ] == 1.0
    assert robot["obstacle_cloud_filter"]["ros__parameters"]["use_sim_time"] is False
    assert "update()" not in callback
    assert "force_update()" not in callback
    for field in (
        "processed_frequency_hz",
        "input_points",
        "after_height",
        "after_self_crop",
        "after_voxel",
        "output_points",
        "processing_latency_ms",
        "last_success_age_s",
    ):
        assert f'"{field}"' in source
    assert "tf_failures_total" not in source


def test_all_obstacle_consumers_share_the_filtered_cloud_and_height_range():
    mapping = (ROOT / "launch" / "mapping.launch.py").read_text()
    nav2 = yaml.safe_load(
        (ROOT / "config" / "nav2_overrides.yaml").read_text()
    )
    robot = yaml.safe_load((ROOT / "config" / "robot.yaml").read_text())

    assert OBSTACLE_TOPIC in mapping
    global_cloud = nav2["global_costmap"]["global_costmap"][
        "ros__parameters"
    ]["obstacle_layer"]["fastlio_cloud"]
    local_cloud = nav2["local_costmap"]["local_costmap"][
        "ros__parameters"
    ]["voxel_layer"]["fastlio_cloud"]
    assert global_cloud["topic"] == OBSTACLE_TOPIC
    assert local_cloud["topic"] == OBSTACLE_TOPIC
    for source in (global_cloud, local_cloud):
        assert source["min_obstacle_height"] == 0.05
        assert source["max_obstacle_height"] == 1.50

    filter_params = robot["obstacle_cloud_filter"]["ros__parameters"]
    assert filter_params["input_topic"] == "/cloud_registered_body"
    assert filter_params["output_topic"] == OBSTACLE_TOPIC
    assert filter_params["target_frame"] == "base_link"


def test_behavior_server_and_arm_refresh_use_standard_local_costmap():
    nav2 = yaml.safe_load(
        (ROOT / "config" / "nav2_overrides.yaml").read_text()
    )
    robot = yaml.safe_load((ROOT / "config" / "robot.yaml").read_text())
    navigator = (ROOT / "vlm_nav" / "vlm_navigator.py").read_text()

    behavior = nav2["behavior_server"]["ros__parameters"]
    assert behavior["costmap_topic"] == "/local_costmap/costmap_raw"
    assert behavior["footprint_topic"] == "/local_costmap/published_footprint"
    assert (
        robot["vlm_nav"]["ros__parameters"]["behavior_costmap_topic"]
        == "/local_costmap/costmap_raw"
    )
    assert '"behavior_costmap_topic": "/local_costmap/costmap_raw"' in navigator
    assert "/vlm_nav/behavior_costmap_raw" not in navigator


def test_package_declares_cpp_dependencies_and_pcl_filters_component():
    package = ET.parse(ROOT / "package.xml").getroot()
    cmake = (ROOT / "CMakeLists.txt").read_text()
    dependencies = {
        element.text
        for element in package.findall("depend")
        if element.text
    }

    assert {"rclcpp", "sensor_msgs", "diagnostic_updater", "message_filters"} <= dependencies
    assert {"tf2", "tf2_ros", "pcl_ros", "pcl_conversions"} <= dependencies
    assert "find_package(PCL REQUIRED COMPONENTS filters)" in cmake
    assert "pcl_filters" not in cmake


def test_every_python_test_is_registered_with_ament_cmake_pytest():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    expected = {
        "test_arm_image_recorder.py",
        "test_easy_case_config.py",
        "test_exploration.py",
        "test_geometry.py",
        "test_latest_frame_worker.py",
        "test_navigation_gate.py",
        "test_pipeline.py",
        "test_vlm_client.py",
        "test_obstacle_chain_config.py",
    }
    registered = set(
        re.findall(r"ament_add_pytest_test\([^\s]+\s+test/([^\s\)]+)", cmake)
    )

    assert registered == expected


def test_runtime_chain_contains_no_behavior_costmap_copy():
    checked = [
        ROOT / "launch" / "vlm_navigation.launch.py",
        ROOT / "config" / "robot.yaml",
        ROOT / "config" / "nav2_overrides.yaml",
        ROOT / "vlm_nav" / "vlm_navigator.py",
    ]

    assert all(
        "/vlm_nav/behavior_costmap_raw" not in path.read_text()
        for path in checked
    )
