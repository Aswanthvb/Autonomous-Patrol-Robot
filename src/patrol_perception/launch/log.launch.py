import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_py = os.path.expanduser('~/yolo_venv/bin/python3')
    default_out = os.path.expanduser('~/detection_runs')
    return LaunchDescription([
        DeclareLaunchArgument('python', default_value=default_py,
                              description='Python of the venv (needs Pillow for snapshots)'),
        DeclareLaunchArgument('output_dir', default_value=default_out,
                              description='Folder for detection runs'),
        ExecuteProcess(
            cmd=[LaunchConfiguration('python'), '-m', 'patrol_perception.detection_logger',
                 '--ros-args', '-p', 'use_sim_time:=true',
                 '-p', ['output_dir:=', LaunchConfiguration('output_dir')]],
            output='screen'),
    ])
