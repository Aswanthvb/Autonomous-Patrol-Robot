import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory("patrol_sim")
    desc_share = get_package_share_directory("patrol_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_file = os.path.join(sim_share, "worlds", "patrol_world.sdf")
    bridge_file = os.path.join(sim_share, "config", "bridge.yaml")
    xacro_file = os.path.join(desc_share, "urdf", "patrol_robot.urdf.xacro")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": f"-r --render-engine-gui ogre {world_file}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "patrol_robot",
                   "-x", x, "-y", y, "-z", "0.01", "-Y", yaw],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_file}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="-2.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        gazebo, rsp, spawn, bridge,
    ])
