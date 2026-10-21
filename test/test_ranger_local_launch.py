"""Evaluate the launch actions without starting any ROS process."""
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch_ros.actions import Node

ROOT=Path(__file__).resolve().parents[1]


def load(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'launch'/name)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod


def context(description,**overrides):
    c=LaunchContext()
    for action in description.entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(c)
    c.launch_configurations.update(overrides)
    return c


def test_legacy_navigation_default_remains_enabled_but_can_be_excluded():
    ld=load('vlm_navigation.launch.py').generate_launch_description()
    legacy=next(a for a in ld.entities if isinstance(a,Node) and a.node_executable=='vlm_navigator')
    c=context(ld)
    assert legacy.condition.evaluate(c)
    c.launch_configurations['start_legacy_navigator']='false'
    assert not legacy.condition.evaluate(c)
    assert all(a.condition is None or a.condition.evaluate(c) for a in ld.entities if isinstance(a,Node) and a is not legacy)


def test_system_forwards_legacy_switch_and_nav2_profile():
    ld=load('system.launch.py').generate_launch_description()
    c=context(ld,start_legacy_navigator='false',nav2_overrides_file='/tmp/ranger-profile.yaml')
    forwarded=[dict(a.launch_arguments) for a in ld.entities if isinstance(a,IncludeLaunchDescription)]
    assert any('start_legacy_navigator' in args and args['start_legacy_navigator'].perform(c)=='false' for args in forwarded)
    assert any('overrides_file' in args and args['overrides_file'].perform(c)=='/tmp/ranger-profile.yaml' for args in forwarded)
