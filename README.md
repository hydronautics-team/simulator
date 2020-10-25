# AUV-ROV_simple_simulator
Simple simulator based on Gazebo for testing control algorithms 

## Building and running
**Warning!** Currently simulator is broken on ROS Noetic. As a workaround, consider to use Docker (see corresponding section).

### Standard way
To build simulator, use standard `catkin_make` command. To run the simulator, use launch file:

```sh
roslaunch Simulation.launch
```

### Docker
Solution for Docker is based on [following instruction](https://answers.ros.org/question/300113/docker-how-to-use-rviz-and-gazebo-from-a-container/). You need to have installed Docker and `nvidia-docker2` runtime, see [section 2 of following instruction](https://cnvrg.io/how-to-setup-docker-and-nvidia-docker-2-0-on-ubuntu-18-04/) to install it (you probably should skip step with re-installing the drivers). Currently, this solutions works only with Nvidia GPU.

Build Docker image with command:
```sh
docker build --rm -t auv-rov-simulator .
```
To run simulator in Docker, run ROS master in separate terminal using `roscore` command or by running some *.launch* files. Then, Run our script:
```sh
./run_docker_gui.sh
```
If everything is ok, simulator window will appear after some seconds. All topics and services from simulator will be accessible from host since simulator uses host's ROS master (see the script source). If you face any problems, first try to see [the original instruction](https://answers.ros.org/question/300113/docker-how-to-use-rviz-and-gazebo-from-a-container/).

## Cameras
UV has 3 cameras:

- 2 front cameras (stereo_vision)
- 1 bottom camera

Cameras' topics:

	/stereo/camera/left/image_raw
	/stereo/camera/right/image_raw
	/ROV_model_URDF/camera_bottom/image_raw

To run cameras write in terminal:

```sh
$ rosrun image_view image_view image:=/stereo/camera/left/image_raw
$ rosrun image_view image_view image:=/stereo/camera/right/image_raw
$ rosrun image_view image_view image:=/ROV_model_URDF/camera_bottom/image_raw
```
To run calibration write in terminal:

```sh
$ rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 0.108 right:=/stereo/camera/right/image_raw left:=/stereo/camera/left/image_raw right_camera:=/stereo/camera/right left_camera:=/stereo/camera/left
```

## Sensors
UV has IMU sensor 

## Custom Gazebo plugins
model_move_plugin was written to move UV. Through publishing twist messages you can control robots's planar movements and its hight. Odom message can inform you about robot's position, orientation and twist parameters.

## Will be added
- Unity3D integration (using this https://github.com/siemens/ros-sharp)
- Depth sensor

## Will be fixed
Bad physics. Buoyancy and hydrodynamic plugins will be fixed.


