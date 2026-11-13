#!/usr/bin/env bash
# Prints the lifecycle state of every Nav2 node and the key transforms.
source /opt/ros/jazzy/setup.bash
[ -f ~/patrol_ws/install/setup.bash ] && source ~/patrol_ws/install/setup.bash
for n in map_server amcl controller_server planner_server smoother_server behavior_server bt_navigator velocity_smoother collision_monitor; do
  printf "%-20s %s\n" "$n" "$(ros2 lifecycle get /$n 2>&1 | head -1)"
done
echo "--- map -> odom"
timeout 5 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -6
echo "--- map -> base_footprint"
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -6
