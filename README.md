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


ROS2 Iron

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

 - `topic:` /cameraaa_image
 - `type:` sensor/msg/Image
 - `description:` видеопоток с камеры
 - 
 - `topic:` /depth_camera
 - `type:` sensor/msg/Image
 - `description:` видеопоток с камеры глубины
 - 
 - `topic:` /depth_camera/points
 - `type:` sensor_msgs/msg/PointCloud2
 - `description:` облако точек к камеры глубины
 - 
 - `topic:` /X3/gazebo/command/twist
 - `type:` geometry_msgs/msg/Twist
 - `description:` управление роботом согласно формату
 - 
 - `topic:` /distance_to_start_zone
 - `type:` Float64
 - `description:` расстояние от стартовой зоны
 - 
 - `topic:` /distance_to_pinger
 - `type:` Float64
 - `description:` расстояние от пингера
 - 
 - `topic:` /angle_to_pinger
 - `type:` Float64
 - `description:` угол 360 градусов относительно носа аппарата по часовой стрелке
 - 
 - `topic:` /depth
 - `type:` Float64
 - `description:` глубина в метрах
 - 
 - `topic:` /distance_to_bottom
 - `type:` Float64
 - `description:` расстояние до дна в метрах
 - 
 - `topic:` /Bbox_array
 - `type:` stingray_interfaces/msg/Bbox
 - `description:` отправка информации о детектировании обьектов данных в формате Данила. отсчёт ведётся с левого верхнего угла картинки
 - 
 - `topic:` /imu
 - `type:` sensor_msgs/msg/Imu
 - `description:` данные с IMU, акселлерометр, гироскоп и реальная ориентация
 - 
 - `topic:` /model/copter/odometry
 - `type:` nav_msgs/msg/odometry
 - `description:` ориентация и реальное положение робота относительно левого нижнего угла бассейна (см. схему бассейна в правилах SAUVC)
 - 
 - `topic:` /camera_info
 - `type:` sensor_msgs/msg/CameraInfo
 - `description:` данные о камерах. Нужно проверять разрешение, иначе какая именно это камера не узнать.
 - 
 - `topic:` /copter/depth
 - `type:` Float64
 - `description:` высота относительно дна, на которой должен оказаться аппарат в метрах
 - 
 - `topic:` /copter/course
 - `type:` Float64
 - `description:` требуемый курс от аппарата, относительно старта по часовой стрелки. Все 360 градусов
 - 
 - `topic:` /SetTwist
 - `type:` stingray_interfaces/srv/SetTwist
 - `description:` pitch и roll игнорируются. 
 - 