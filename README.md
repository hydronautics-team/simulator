# Hydronautics simulator
Simple AUV simulator based on [Field-Robotics-Lab/uuv_simulator](https://github.com/Field-Robotics-Lab/uuv_simulator) for testing control algos

## Building

1. 	
    ```sh
    git submodule update --init --recursive
    ```
2.
    ```sh
    catkin_make
    ```

## Running

To run the simulator, use launch file:
```sh
source devel/setup.bash
roslaunch Simulation.launch
```

## Cameras
UV has 3 cameras:

- 2 front cameras (stereo_vision)
- 1 bottom camera

Cameras' topics:

	/stereo/camera/left/image_raw
	/stereo/camera/right/image_raw
	/rov_model_urdf/camera_bottom/image_raw

To run cameras write in terminal:

```sh
$ rosrun image_view image_view image:=/stereo/camera/left/image_raw
$ rosrun image_view image_view image:=/stereo/camera/right/image_raw
$ rosrun image_view image_view image:=/rov_model_urdf/camera_bottom/image_raw
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
- Depth sensor

## Will be fixed
Bad physics. Buoyancy and hydrodynamic plugins will be fixed.


