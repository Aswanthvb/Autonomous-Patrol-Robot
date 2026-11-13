#!/usr/bin/env bash
# Installs Step 4 (SLAM mapping) files into ~/patrol_ws/src/patrol_nav
set -e
SRC="$HOME/patrol_ws/src"
[ -d "$SRC/patrol_nav" ] || { echo "patrol_nav package not found."; exit 1; }

mkdir -p "$(dirname "$SRC/patrol_nav/package.xml")"
cat > "$SRC/patrol_nav/package.xml" <<'PATROL_EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>patrol_nav</name>
  <version>0.1.0</version>
  <description>SLAM, localization and Nav2 configuration for the patrol robot</description>
  <maintainer email="aswanthvb009@gmail.com">aswanth-v-b</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
PATROL_EOF
mkdir -p "$(dirname "$SRC/patrol_nav/CMakeLists.txt")"
cat > "$SRC/patrol_nav/CMakeLists.txt" <<'PATROL_EOF'
cmake_minimum_required(VERSION 3.8)
project(patrol_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME})

ament_package()
PATROL_EOF
mkdir -p "$(dirname "$SRC/patrol_nav/maps/.gitkeep")"
cat > "$SRC/patrol_nav/maps/.gitkeep" <<'PATROL_EOF'

PATROL_EOF
mkdir -p "$(dirname "$SRC/patrol_nav/config/slam_toolbox_mapping.yaml")"
cat > "$SRC/patrol_nav/config/slam_toolbox_mapping.yaml" <<'PATROL_EOF'
slam_toolbox:
  ros__parameters:
    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS params
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    use_map_saver: true
    mode: mapping

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 2.0
    resolution: 0.05
    min_laser_range: 0.12
    max_laser_range: 8.0
    minimum_time_interval: 0.5
    transform_timeout: 0.5
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    enable_interactive_mode: true

    # Scan matching and motion thresholds
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.2
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5

    # Loop closure
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan-matcher penalties
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
PATROL_EOF
mkdir -p "$(dirname "$SRC/patrol_nav/launch/slam.launch.py")"
cat > "$SRC/patrol_nav/launch/slam.launch.py" <<'PATROL_EOF'
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav_share = get_package_share_directory("patrol_nav")
    slam_share = get_package_share_directory("slam_toolbox")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "online_async_launch.py")),
        launch_arguments={
            "slam_params_file": os.path.join(nav_share, "config", "slam_toolbox_mapping.yaml"),
            "use_sim_time": "true",
        }.items(),
    )
    return LaunchDescription([slam])
PATROL_EOF
echo "Done. Next: cd ~/patrol_ws && colcon build --symlink-install && source install/setup.bash"
