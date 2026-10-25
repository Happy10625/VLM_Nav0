import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vlm_nav.fixed_map import validate_localization_mode, validate_map_yaml


def scan_converter_node():
    return Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="fastlio_cloud_to_scan",
        output="screen",
        remappings=[
            ("cloud_in", "/vlm_nav/obstacle_cloud"),
            ("scan", "/scan"),
        ],
        parameters=[
            {
                "target_frame": "base_link",
                "transform_tolerance": 0.10,
                "min_height": 0.10,
                "max_height": 1.60,
                "angle_min": -3.141592653589793,
                "angle_max": 3.141592653589793,
                "angle_increment": 0.008726646259972,
                "scan_time": 0.10,
                "range_min": 0.50,
                "range_max": 20.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
    )


def configure_localization(context):
    mode = validate_localization_mode(
        LaunchConfiguration("localization_mode").perform(context)
    )
    share = get_package_share_directory("vlm_nav")
    if mode == "slam":
        return [
            scan_converter_node(),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[os.path.join(share, "config", "slam_toolbox.yaml")],
            )
        ]

    map_yaml, map_image = validate_map_yaml(
        LaunchConfiguration("map").perform(context)
    )
    scan_topic = LaunchConfiguration("amcl_scan_topic").perform(context).strip()
    if not scan_topic:
        raise ValueError("amcl_scan_topic must not be empty")
    amcl_params = LaunchConfiguration("amcl_params_file").perform(context)
    if not os.path.isfile(amcl_params):
        raise ValueError(f"AMCL parameter file does not exist: {amcl_params}")
    print(f"Validated fixed map YAML: {map_yaml}")
    print(f"Validated fixed map image: {map_image}")
    return [
        scan_converter_node(),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[amcl_params, {"yaml_filename": str(map_yaml), "use_sim_time": False}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[amcl_params, {"scan_topic": scan_topic, "use_sim_time": False}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "autostart": True,
                    "node_names": ["map_server", "amcl"],
                }
            ],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory("vlm_nav")
    return LaunchDescription(
        [
            DeclareLaunchArgument("localization_mode", default_value="slam"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("amcl_scan_topic", default_value="/scan"),
            DeclareLaunchArgument(
                "amcl_params_file",
                default_value=os.path.join(share, "config", "ranger_amcl.yaml"),
            ),
            OpaqueFunction(function=configure_localization),
        ]
    )
