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
        default_value=os.path.join(usenav_description_dir, "urdf", "usenav.urdf.xacro"),
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

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            # *** FIX: Map Gazebo joints directly to standard ROS topic ***
            "/world/empty/model/usenav/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/model/usenav/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/joint1_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/joint2_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/joint3_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/joint4_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/joint5_cmd@std_msgs/msg/Float64@gz.msgs.Double",
        ],
        # Remap the long Gazebo topic to standard /joint_states
        remappings=[("/world/empty/model/usenav/joint_state", "/joint_states")],
        output="screen",
    )

    ld.add_action(model_arg)
    ld.add_action(rsp_node)
    ld.add_action(gazebo_resource_path)
    ld.add_action(gazebo)
    ld.add_action(gz_spawn_enity_node)
    ld.add_action(gz_bridge)

    return ld
