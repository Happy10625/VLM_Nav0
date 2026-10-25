"""Start Nav2 with upstream defaults merged with this robot's overrides."""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vlm_nav.fixed_map import validate_localization_mode


def deep_merge(destination, source):
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(destination.get(key), dict):
            deep_merge(destination[key], value)
        else:
            destination[key] = value


def configure_navigation(context):
    bringup = get_package_share_directory("nav2_bringup")
    default_params = os.path.join(bringup, "params", "nav2_params.yaml")
    mode = validate_localization_mode(
        LaunchConfiguration("localization_mode").perform(context)
    )
    overrides = LaunchConfiguration("overrides_file").perform(context).strip()
    if not overrides:
        overrides = LaunchConfiguration(
            "amcl_overrides_file" if mode == "amcl" else "slam_overrides_file"
        ).perform(context)
    with open(default_params, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    with open(overrides, encoding="utf-8") as stream:
        deep_merge(params, yaml.safe_load(stream))
    keepout=LaunchConfiguration('keepout_mask_yaml').perform(context).strip()
    servers=[]
    if keepout:
        from vlm_nav.keepout import KeepoutMask
        mask=KeepoutMask(LaunchConfiguration('map').perform(context),keepout)
        if abs(mask.origin[2])>1e-9:raise ValueError('Humble Keepout requires an axis-aligned map (origin yaw=0)')
        expected=LaunchConfiguration('keepout_mask_digest').perform(context).strip()
        if expected and mask.digest!=expected:raise ValueError('keepout digest differs from startup configuration')
        for name in ('global_costmap','local_costmap'):
            costmap=params[name][name]['ros__parameters']
            filters=costmap.setdefault('filters',[])
            if 'keepout_filter' not in filters:filters.append('keepout_filter')
            costmap['keepout_filter']=dict(plugin='nav2_costmap_2d::KeepoutFilter',enabled=True,
                filter_info_topic='/costmap_filter_info',transform_tolerance=.1)
        servers=[
            Node(package='nav2_map_server',executable='map_server',name='keepout_mask_server',output='screen',
                parameters=[dict(use_sim_time=False,yaml_filename=str(mask.path),topic_name='/keepout_filter_mask',frame_id='map')]),
            Node(package='nav2_map_server',executable='costmap_filter_info_server',name='costmap_filter_info_server',output='screen',
                parameters=[dict(use_sim_time=False,type=0,filter_info_topic='/costmap_filter_info',
                    mask_topic='/keepout_filter_mask',base=0.0,multiplier=1.0)]),
            Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_keepout',output='screen',
                parameters=[dict(use_sim_time=False,autostart=True,node_names=['keepout_mask_server','costmap_filter_info_server'])])]
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
    if params.get('controller_server',{}).get('ros__parameters',{}).get('FollowPath',{}).get('plugin') == 'vlm_nav::RangerDWBController':
        params['bt_navigator']['ros__parameters']['plugin_lib_names'].append('ranger_recovery_bt_node')
        # Ranger's generator is the sole acceleration owner; this graph has no velocity_smoother.
        components=(('nav2_controller','controller_server'),('nav2_smoother','smoother_server'),
                    ('nav2_planner','planner_server'),('nav2_behaviors','behavior_server'),
                    ('nav2_bt_navigator','bt_navigator'),('nav2_waypoint_follower','waypoint_follower'))
        params.pop('velocity_smoother',None)
        with open(handle.name,'w',encoding='utf-8') as stream:yaml.safe_dump(params,stream,sort_keys=False)
        return servers+[
            Node(package='vlm_nav',executable='ranger_command_guard',name='ranger_command_guard',
                output='screen',parameters=[handle.name])
        ]+[
            Node(package=package,executable=name,name=name,output='screen',parameters=[handle.name],
                remappings=[('/tf','tf'),('/tf_static','tf_static')]+
                    ([('cmd_vel','cmd_vel_nav')] if name in ('controller_server','behavior_server') else []))
            for package,name in components
        ]+[
            Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',
                output='screen',parameters=[dict(use_sim_time=False,autostart=True,
                    node_names=[name for _,name in components])])
        ]
    return servers+[
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
                default_value="",
            ),
            DeclareLaunchArgument("localization_mode", default_value="slam"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("keepout_mask_yaml", default_value=""),
            DeclareLaunchArgument("keepout_mask_digest", default_value=""),
            DeclareLaunchArgument(
                "slam_overrides_file",
                default_value=os.path.join(share, "config", "nav2_overrides.yaml"),
            ),
            DeclareLaunchArgument(
                "amcl_overrides_file",
                default_value=os.path.join(
                    share, "config", "nav2_amcl_overrides.yaml"
                ),
            ),
            OpaqueFunction(function=configure_navigation),
        ]
    )
