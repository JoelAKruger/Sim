"""Regolith with a robot from xacro or URDF.

    ros2 launch regolith sim.launch.py description:=/path/to/rover.urdf.xacro
    ros2 launch regolith sim.launch.py description:=... headless:=true config:=my.yaml

robot_state_publisher publishes the expanded URDF on /robot_description (latched), and
the sim builds the robot from it once, at startup. Everything runs on the system clock,
or with ros.use_sim_time, on sim time from Regolith's /clock.
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _uses_sim_time(config):
    """ros.use_sim_time from the settings file, which defaults to false."""
    if not config:
        return False
    with open(os.path.expanduser(config)) as file:
        settings = yaml.safe_load(file) or {}
    settings = settings.get("regolith", {}).get("ros__parameters", settings)
    return bool(settings.get("ros", {}).get("use_sim_time", False))


def _nodes(context):
    description = LaunchConfiguration("description").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() in ("true", "1", "yes")
    config = LaunchConfiguration("config").perform(context)

    nodes = []
    if description:
        # xacro.process_file handles plain URDF too.
        urdf = xacro.process_file(os.path.expanduser(description)).toxml()
        nodes.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf, "use_sim_time": _uses_sim_time(config)}],
            output="screen",
        ))
    nodes.append(Node(
        package="regolith",
        executable="regolith",
        arguments=["--headless"] if headless else [],
        parameters=[config] if config else [],
        output="screen",
        emulate_tty=True,
    ))
    return nodes


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("regolith"), "config", "sim.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "description", default_value="",
            description="xacro or URDF file for robot_state_publisher; empty to use "
                        "robot.urdf from the config, or another /robot_description publisher"),
        DeclareLaunchArgument("headless", default_value="false", description="run without a window"),
        DeclareLaunchArgument("config", default_value=default_config,
                              description="Regolith settings (ROS params YAML)"),
        OpaqueFunction(function=_nodes),
    ])
