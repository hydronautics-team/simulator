# Stingray-compatible simulation

This package exposes any supported world using the same ROS API as `stingray_core`.
No remapping is required by mission, planning or object-detection nodes.

Build and run inside the simulator container:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch stingray_sim stingray_world.launch.py
```

Or run the host helper (it uses `ROS_DOMAIN_ID=1`, like `welt_auv`):

```bash
./run-stingray-sim.sh
```

The launch publishes `/clock`, camera, IMU, pressure, odometry and DVL topics.
It consumes `/control/data`, `/control/mode` and `/control/loop_flags`. The
initial vehicle position becomes `(0, 0, 0)` in `odom`; TF is
`map -> odom -> base_link`.

Quick smoke test:

```bash
ros2 topic hz /stingray/topics/camera/front
ros2 topic echo --once /core/state/odometry
ros2 topic pub --once /control/mode std_msgs/msg/UInt8 '{data: 2}'
ros2 topic pub --once /control/data geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 20.0}}'
```
