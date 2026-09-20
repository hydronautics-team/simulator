#!/bin/bash
set -e

# ROS 2 underlay installed with the image (lyrical, see ROS_DISTRO in the
# Dockerfile).
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Workspace overlay built from the repository (docker-compose mounts the repo
# at /ws/src/simulator). When it is missing the workspace is built once.
if [ -f /ws/src/simulator/install/setup.bash ]; then
  source /ws/src/simulator/install/setup.bash
else
  echo "[INFO] /ws/src/simulator/install/setup.bash not found, building..."
  cd /ws/src/simulator && colcon build
  source /ws/src/simulator/install/setup.bash
fi

exec "$@"
