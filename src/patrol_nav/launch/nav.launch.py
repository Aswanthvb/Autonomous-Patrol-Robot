import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory("patrol_nav")
    bringup_share = get_package_share_directory("nav2_bringup")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": "true",
            "autostart": "true",
            "slam": "False",
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(bringup_share, "rviz", "nav2_default_view.rviz")],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(nav_share, "maps", "patrol_map.yaml")),
        DeclareLaunchArgument("params_file", default_value=os.path.join(nav_share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("rviz", default_value="true"),
        nav2,
        rviz_node,
    ])
