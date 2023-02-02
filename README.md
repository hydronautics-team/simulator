# Hydronautics simulator
Hydronautics Gazebo simulation for [SAUVC](https://github.com/hidronautics/sauvc) and `TEKNOFEST` based on [uuv_simulator](https://github.com/hydronautics-team/uuv_simulator) for testing control algorithms

## Run

### SAUVC
```bash
sudo ./run_sauvc.sh
```

### TEKNOFEST

Build docker image:
```bash
sudo docker build -t hydronautics/simulator:teknofest -f Dockerfile.teknofest .
```

Run docker container:
```bash
sudo ./run_teknofest.sh
```

## Development

### Build
1. 	
    ```sh
    git submodule update --init --recursive
    ```
2.
    ```sh
    source /opt/ros/noetic/setup.bash
    catkin_make
    ```

### Running

To run the simulator, use launch file:
```sh
source devel/setup.bash
roslaunch Simulation.launch
```
### Run rtabmap mapping

```bash
roslaunch rtabmap_ros rtabmap.launch \
rtabmap_args:="--delete_db_on_start" \
rgb_topic:=/rov_model_urdf/camera_depth/color/image_raw \
depth_topic:=/rov_model_urdf/camera_depth/depth/image_raw \
camera_info_topic:=/rov_model_urdf/camera_depth/color/ \
camera_info frame_id:=camera_depth approx_sync:=false \
rgbd_sync:=true
```

### Cameras
UV has 5 cameras:

- 1 front camera 
- 1 front depth camera 
- 1 bottom camera

Cameras' topics:
	/rov_model_urdf/camera_bottom/image_raw
	/rov_model_urdf/camera_front/image_raw
	/rov_model_urdf/camera_depth/color/image_raw
	/rov_model_urdf/camera_depth/depth/image_raw

To run depth camera visualization write in terminal:

```sh
rosrun rviz rviz
```

### Sensors
UV has IMU sensor 

### Custom Gazebo plugins
`model_move_plugin` was written to move UV. Through publishing twist messages you can control robots's planar movements and its hight. Odom message can inform you about robot's position, orientation and twist parameters.

### Will be added
- Depth sensor

### Will be fixed
Bad physics. Buoyancy and hydrodynamic plugins will be fixed.


