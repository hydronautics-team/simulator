#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

ros_domain_id="${ROS_DOMAIN_ID:-1}"
rebuild=false
if [[ "${1:-}" == "--build" ]]; then
  rebuild=true
elif [[ $# -gt 0 ]]; then
  echo "Usage: $0 [--build]" >&2
  exit 2
fi

docker compose up -d simulator
# Restarting kills any stale gz-sim process but preserves /ws/build and
# /ws/install in the container, so a normal run does not rebuild the workspace.
docker compose restart simulator >/dev/null

if [[ "$rebuild" == "true" ]]; then
  docker compose exec simulator bash -lc \
    'cd /ws && source /opt/ros/lyrical/setup.bash && colcon build --symlink-install --packages-select descriptions gazebo_worlds gazebo_plugins stingray_sim'
elif ! docker compose exec -T simulator test -f /ws/install/stingray_sim/share/stingray_sim/package.sh; then
  echo "Simulator workspace is not built yet; performing the initial build."
  docker compose exec simulator bash -lc \
    'cd /ws && source /opt/ros/lyrical/setup.bash && colcon build --symlink-install --packages-select descriptions gazebo_worlds gazebo_plugins stingray_sim'
fi

docker compose exec -e ROS_DOMAIN_ID="$ros_domain_id" simulator bash -lc \
  'cd /ws && source /opt/ros/lyrical/setup.bash && source install/setup.bash && ros2 launch stingray_sim sauvc_2027.launch.py gui:='"${SIM_GUI:-true}"
