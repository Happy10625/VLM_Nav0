"""Start Nav2 with upstream defaults merged with this robot's overrides."""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def deep_merge(destination, source):
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(destination.get(key), dict):
            deep_merge(destination[key], value)
        else:
            destination[key] = value


def configure_navigation(context):
    bringup = get_package_share_directory("nav2_bringup")
    default_params = os.path.join(bringup, "params", "nav2_params.yaml")
    overrides = LaunchConfiguration("overrides_file").perform(context)
    with open(default_params, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    with open(overrides, encoding="utf-8") as stream:
        deep_merge(params, yaml.safe_load(stream))
    bt_parameters = params.get("bt_navigator", {}).get("ros__parameters", {})
    bt_xml = bt_parameters.get("default_nav_to_pose_bt_xml", "")
    package_prefix = "package://vlm_nav/"
    if isinstance(bt_xml, str) and bt_xml.startswith(package_prefix):
        bt_parameters["default_nav_to_pose_bt_xml"] = os.path.join(
            get_package_share_directory("vlm_nav"),
            bt_xml[len(package_prefix):],
        )
    for server in (
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ):
        if server in params:
            params[server].setdefault("ros__parameters", {})["use_sim_time"] = False
    handle = tempfile.NamedTemporaryFile(
        mode="w", prefix="vlm_nav2_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(params, handle, sort_keys=False)
    handle.close()
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "params_file": handle.name,
                "use_sim_time": "false",
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        )
    ]


def generate_launch_description():
    share = get_package_share_directory("vlm_nav")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "overrides_file",
                default_value=os.path.join(share, "config", "nav2_overrides.yaml"),
            ),
            OpaqueFunction(function=configure_navigation),
        ]
    )
