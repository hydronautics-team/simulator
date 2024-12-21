# Hydronautics simulator
Hydronautics Gazebo simulation for SAUVC 2025

## Run

### SAUVC
```bash
ros2 launch ros_gz_example_bringup NeedToBelieve.launch.py
```

## Dependencies

Ignition Fortress
[Link](https://gazebosim.org/docs/fortress/install/)

## Development

### Build
1. 	
	Установка моста между ROS2 и Gazebo
   ```sh
    sudo apt install ros-<distro>-ros-gz
    ```
3.
    ```sh
    colcon build
    ```

### Topics description

- `topic:` /imu
- `type:` sensor_msgs/msg/Imu
- `description:` данные с IMU, реальная позиция, акселлерометр и гироскоп
- 
- `topic:` /depth
- `type:` Float64
- `description:` расстояние до дна
- 
- `topic:` /For_Danil
- `type:` stingray_interfaces/msg/Bbox
- `description:` отправка информации о детектировании обьектов данных в формате Данила
- 
- `topic:` /cameraaa_image
- `type:` sensor/msg/Image
- `description:` видеопоток с камеры
- 
- `topic:` /X3/gazebo/command/twist
- `type:` geometry_msgs/msg/Twist
- `description:` управление роботом


