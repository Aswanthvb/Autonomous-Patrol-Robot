from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('laps', default_value='-1',
                              description='Number of laps; -1 uses the file, 0 = forever'),
        DeclareLaunchArgument('output_dir', default_value='',
                              description='Log folder; empty uses the file value'),
        Node(
            package='patrol_task_manager',
            executable='patrol_manager',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'laps': ParameterValue(LaunchConfiguration('laps'), value_type=int),
                'output_dir': ParameterValue(LaunchConfiguration('output_dir'), value_type=str),
            }],
        ),
    ])
