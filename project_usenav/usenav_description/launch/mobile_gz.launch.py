import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    usenav_description_dir = get_package_share_directory("usenav_description")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(usenav_description_dir, "urdf", "mobile","standalone_mobile.xacro"),
        description="Absolute path to robot URDF file.",
    )

    robot_description = ParameterValue(
        Command(
            ["xacro ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]
        ),
        value_type=str,
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(usenav_description_dir).parent.resolve())],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(ros_gz_sim_dir, "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        launch_arguments=[("gz_args", [" -v 4", " -r", " empty.sdf"])],
    )

    gz_spawn_enity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "usenav"],
    )

    ld.add_action(model_arg)
    ld.add_action(rsp_node)
    ld.add_action(gazebo_resource_path)
    ld.add_action(gazebo)
    ld.add_action(gz_spawn_enity_node)

    return ld