#!/usr/bin/env python3

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_path
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    urdf_path = os.path.join(
        get_package_share_path("usenav_description"), "urdf", "usenav.urdf.xacro"
    )

    display_rviz_path = os.path.join(
        get_package_share_path("usenav_description"), "config", "display.rviz"
    )

    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui"
    )

    rviz_args = []
    if os.path.isfile(display_rviz_path):
        rviz_args += ["-d", display_rviz_path]
    else:
        print(f"==> [usenav_rviz.launch.py] [WARN] RViz config not found: {display_rviz_path}")

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=rviz_args,
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz2)

    return ld
