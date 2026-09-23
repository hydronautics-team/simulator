# AGENTS.md

ROS 2 Lyrical + Gazebo Sim Noble (gz-sim 10) workspace for testing an underwater buoyancy/hydrodynamics plugin. All packages are ament_cmake; build with colcon from the repo root. The supported environment is Docker (`Dockerfile` at the repo root builds a `osrf/ros:lyrical-simulation`-based image); the code no longer builds against Humble/Ignition Fortress.

## Layout — read this before touching anything

- `gazebo_plugins/gazebo_plugins/` — the ACTIVE plugin package. Builds `libunderwater_object.so` (buoyancy + hydrodynamics, gz-sim system plugin name `underwater_object`), `libthruster.so` (plugin name `thruster`) and `libwater_pressure.so` (water pressure / depth sensor, plugin name `water_pressure`). Sources in `src/`, headers in `inc/`. C++17.
- `gazebo_plugins/gazebo_plugins/legacy_classic/` — former standalone plugin, kept for reference only, NOT built (see CMakeLists note).
- `gazebo_plugins/uuv_gazebo_ros_plugins{,_msgs}/` and the whole `uuv_gazebo_plugins/` tree — vendored UUV Simulator reference code, all COLCON_IGNOREd and NOT built. Do not edit; if code looks duplicated, the `gazebo_plugins/gazebo_plugins` version is the one that matters.
- `descriptions/` — robot description (`robots/ball.xacro`), spawn launch file, keyboard teleop, pytest suite.
- `gazebo_worlds/` — `worlds/{default,empty_underwater}.world`, models `ocean_surface` and `sea_floor`.
- `simulator/` — empty meta package (dependencies only).
- Root-level `Dockerfile` / `docker-compose.yml` / `ros_entrypoint.sh` / `.dockerignore` — Docker tooling (see the header comments in `docker-compose.yml`). The image installs the ROS 2 / Gazebo stack and copies the workspace as a fallback; compose bind-mounts the repo over `/ws/src/simulator`, where the workspace is built inside the container.
- `build/`, `install/`, `log/` — colcon artifacts, COLCON_IGNOREd.
- Root-level `HydrodynamicModel.cpp` is a stray copy and differs from the real source in `gazebo_plugins/gazebo_plugins/src/`. Root-level `*.txt`, `*.pid`, `*.log` (f1.txt, g0.txt, bridge_f2.log, teleop.log, bf2.pid, ...) are experiment scratch, not source.

## Commands

All of these run inside the container (`docker compose exec simulator ...` or
`docker compose run --rm simulator ...`):

```bash
docker compose build                      # build the image (ROS/Gazebo deps)
docker compose up -d                      # start a dev container (X11 passthrough)
docker compose exec simulator bash        # shell inside the container
colcon build                              # from /ws/src/simulator; logs land in log/ (latest_* symlinks)
colcon build --packages-select gazebo_plugins
colcon test --packages-select gazebo_plugins descriptions
colcon test-result --verbose              # readable summary
source install/setup.bash                 # /opt/ros/lyrical and this install are sourced by the entrypoint
```

- Plugin unit tests are gtest (`test_buoyancy_model`, `test_hydrodynamic_model`, `test_propeller`, `test_thruster_converter`); `descriptions` tests are pytest (`test_urdf_files.py` expands xacro, `test_ball_teleop.py` tests mixing logic).
- `gazebo_plugins` also has two launch-based integration tests (`test_underwater_object_integration`, `test_thruster_integration`, TIMEOUT 300). They run `gz sim -s -r` headless and take minutes — run them explicitly, not in a hurry.

## Running the sim

```bash
# inside the container (see the Commands section)
ros2 launch gazebo_worlds empty_underwater_world.launch.py       # start world first
ros2 launch descriptions upload_rexrov_default.launch.py         # spawn the robot
ros2 run descriptions ball_teleop.py --ros-args -p name:=ball    # keyboard teleop (separate terminal)
```

Thruster control via gz topics bridged to ROS 2 (`thrusters:=true`): `/ball/thrusters/id_0/input` (std_msgs Float64, rotor rad/s, thrust = rotorConstant * |w| * w) and `/ball/thrusters/id_0/thrust` (Vector3). Equal speeds = forward, opposite = yaw. The robot also has a built-in IMU (`gz-sim-imu-system` in `ball.xacro`) on gz `/<name>/sensors/imu`, a front camera (`gz-sim-sensors-system` + `<sensor type="camera">`) and a water pressure / depth sensor (`libwater_pressure.so`, hydrostatic P = P_atm + rho*g*depth), all bridged to ROS 2 by the spawn launch: `/<name>/sensors/imu` as `sensor_msgs/Imu`, `/<name>/sensors/camera/front` as `sensor_msgs/Image` with `/<name>/sensors/camera/front/camera_info` as `sensor_msgs/CameraInfo` (the camera_info topic is set explicitly because gz-sensors otherwise derives it by dropping the last path segment of the image topic), and `/<name>/sensors/pressure` as `sensor_msgs/FluidPressure`.

## Gotchas

- Models spawned at runtime through the `create` service do NOT load their gz-sim system plugins. That is why `default.world` embeds the `buoyancy_body` model directly, and why the spawn launch runs the `create` node of `ros_gz_sim` with the xacro-expanded URDF (plugin blocks survive URDF→SDF conversion; `GzSpawnModel` is not used because it forwards None for the arguments it was not given).
- `ball.xacro` is the single source of truth for the robot. Link names are namespace-prefixed (`<name>/base_link`); the spawn entity name must equal the xacro `namespace` arg or the plugin topics won't match.
- Sensor topics follow the `/<name>/sensors/...` naming (as defined by the robot namespace): `/<name>/sensors/imu`, `/<name>/sensors/camera/front` with `/<name>/sensors/camera/front/camera_info`, `/<name>/sensors/pressure`. Keep this pattern for new sensors: the xacro sets `<topic>/${namespace}/sensors/...</topic>` and the spawn launch bridges it with the `name` launch arg. For camera sensors always set `<camera_info_topic>` explicitly (gz-sensors otherwise derives it by dropping the last path segment of the image topic).
- Debug tooling is gated by the spawn launch argument `debug:=true` and debug signals live under `/<name>/debug/...`: the ground truth model pose (xacro-gated `gz-sim-pose-publisher-system` on `/<name>/debug/pose`, bridged as PoseStamped), the in-scene resultant-thrust marker (`gazebo_worlds/scripts/debug_markers.py`, publishes `gz.msgs.Marker` on `/<name>/debug/marker` **and** mirrors it to `/marker` because the built-in MarkerManager of gz sim subscribes to `/marker` only), and live matplotlib windows for the perspectives requested with `perspectives:=<name>[,<name>]` (`gazebo_worlds/scripts/debug_plot.py`, one subplot per topic; configs `descriptions/config/plots/*.yaml` carry `window_title`, `window_seconds`, `update_rate` and the `topics` list where `{name}` is the robot name). PlotJuggler/rqt_plot are not used (no packaged live ROS 2 plugin for PlotJuggler, rqt_plot was dropped in favour of the matplotlib node).
- Duplicate publishers on the same ROS topic (a leftover teleop/bridge from an earlier launch, or a second simulator in the same ROS domain — every container shares the host network) interleave their values and make the plots spike to zero; `debug_plot.py` logs a warning with the publisher names when that happens. Use a separate `ROS_DOMAIN_ID` for parallel test runs.
- The buoyancy model implements *neutral* buoyancy: fluid density is derived as m/V, so changing mass keeps the body neutrally buoyant (explained in the xacro header).
- Launch files and scripts carry long header docstrings documenting usage/physics — keep them in sync when changing behavior.
- Agent skills for planning/review workflows live in `.agents/skills/`; `.agents/skills/ask-matt/SKILL.md` is the router over them.
