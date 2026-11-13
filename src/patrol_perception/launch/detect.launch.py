import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_py = os.path.expanduser('~/yolo_venv/bin/python3')
    return LaunchDescription([
        DeclareLaunchArgument('python', default_value=default_py,
                              description='Python of the virtual environment that has ultralytics'),
        DeclareLaunchArgument('conf', default_value='0.35'),
        DeclareLaunchArgument('max_rate', default_value='2.0',
                              description='Maximum detections per second'),
        # Runs the node with the venv's Python so ultralytics and torch are found.
        ExecuteProcess(
            cmd=[LaunchConfiguration('python'), '-m', 'patrol_perception.yolo_node',
                 '--ros-args', '-p', 'use_sim_time:=true',
                 '-p', ['conf:=', LaunchConfiguration('conf')],
                 '-p', ['max_rate_hz:=', LaunchConfiguration('max_rate')]],
            output='screen'),
    ])
