from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_easy_case_is_opt_in_and_normal_profile_stays_disabled():
    robot = yaml.safe_load((ROOT / "config" / "robot.yaml").read_text())

    assert robot["vlm_nav"]["ros__parameters"]["easy_case_mode"] is False


def test_easy_case_costmaps_are_blank_and_speed_limited():
    config = yaml.safe_load(
        (ROOT / "config" / "nav2_easy_case.yaml").read_text()
    )
    global_params = config["global_costmap"]["global_costmap"][
        "ros__parameters"
    ]
    local_params = config["local_costmap"]["local_costmap"][
        "ros__parameters"
    ]
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]

    assert global_params["plugins"] == []
    assert local_params["plugins"] == []
    assert global_params["track_unknown_space"] is False
    assert local_params["track_unknown_space"] is False
    assert controller["max_vel_x"] == 0.10
    assert controller["max_vel_theta"] == 0.20


def test_easy_case_behavior_tree_has_no_recovery_actions():
    path = ROOT / "behavior_trees" / "easy_case_no_recovery.xml"
    root = ET.parse(path).getroot()
    tags = {element.tag for element in root.iter()}

    assert "ComputePathToPose" in tags
    assert "FollowPath" in tags
    assert not any("Recovery" in tag for tag in tags)


def test_rviz_uses_live_fastlio_cloud_with_visible_defaults():
    config = yaml.safe_load((ROOT / "config" / "vlm_nav.rviz").read_text())
    displays = config["Visualization Manager"]["Displays"]
    point_clouds = [
        item
        for item in displays
        if item["Class"] == "rviz_default_plugins/PointCloud2"
    ]

    assert len(point_clouds) == 1
    cloud = point_clouds[0]
    assert cloud["Enabled"] is True
    assert cloud["Topic"]["Value"] == "/cloud_registered_body"
    assert cloud["Topic"]["Reliability Policy"] == "Reliable"
    assert cloud["Color Transformer"] == "FlatColor"
    assert cloud["Color"] == "255; 128; 0"
    assert "/Laser_map" not in (ROOT / "config" / "vlm_nav.rviz").read_text()


def test_rviz_keeps_core_views_on_and_auxiliary_views_off():
    config = yaml.safe_load((ROOT / "config" / "vlm_nav.rviz").read_text())
    manager = config["Visualization Manager"]
    displays = {item["Name"]: item for item in manager["Displays"]}

    for name in (
        "Map",
        "ROBOT (green arrow)",
        "2D Obstacles (/scan, cyan)",
        "FAST_LIO Live Body Cloud (/cloud_registered_body)",
        "Nav2 Global Plan",
        "VLM TARGET / PATH (red / green)",
        "VLM Annotated Image",
    ):
        assert displays[name]["Enabled"] is True

    for name in (
        "Grid",
        "TF",
        "Nav2 Local Plan",
        "Camera RGB (live)",
        "VLM Frontier Map",
        "VLM Scan Montage",
    ):
        assert displays[name]["Enabled"] is False

    panel_names = {item["Name"] for item in config["Panels"]}
    assert "Selection" not in panel_names
    assert "Tool Properties" not in panel_names

    tool_classes = {item["Class"] for item in manager["Tools"]}
    assert "rviz_default_plugins/SetInitialPose" not in tool_classes
    assert "rviz_default_plugins/PublishPoint" not in tool_classes


def test_mapping_and_nav2_use_the_sparse_filtered_cloud():
    mapping_source = (ROOT / "launch" / "mapping.launch.py").read_text()
    nav2 = yaml.safe_load((ROOT / "config" / "nav2_overrides.yaml").read_text())
    expected_topic = "/vlm_nav/filtered_obstacle_cloud"

    assert expected_topic in mapping_source
    assert "/cloud_registered_body" not in mapping_source
    global_cloud = nav2["global_costmap"]["global_costmap"]["ros__parameters"][
        "obstacle_layer"
    ]["fastlio_cloud"]
    local_cloud = nav2["local_costmap"]["local_costmap"]["ros__parameters"][
        "voxel_layer"
    ]["fastlio_cloud"]
    assert global_cloud["topic"] == expected_topic
    assert local_cloud["topic"] == expected_topic
