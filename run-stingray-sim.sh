#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

ros_domain_id="${ROS_DOMAIN_ID:-1}"
# A stopped launch can leave gz-sim processes alive in the long-running
# development container. Recreate it so every simulation starts with one clean
# world and a fresh local (0, 0) state origin.
docker compose up -d --force-recreate simulator
docker compose exec -e ROS_DOMAIN_ID="$ros_domain_id" simulator bash -lc \
  'cd /ws && source /opt/ros/lyrical/setup.bash && colcon build --symlink-install --packages-select descriptions gazebo_worlds gazebo_plugins stingray_sim && source install/setup.bash && ros2 launch stingray_sim sauvc_2027.launch.py gui:='"${SIM_GUI:-true}"
