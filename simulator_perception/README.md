# simulator_perception

`simulator_perception` — Python-пакет ROS2 для адаптации данных симуляции в сообщения и сервисы стека Stingray.

## Назначение

Пакет:
- принимает данные из симуляции (детекции, камеры, одометрия, IMU);
- преобразует их в формат `stingray_interfaces`;
- публикует состояние и команды в нужные топики;
- предоставляет сервисы управления.

## Основные узлы

- `simulator_perception_node.py`  
  Главный узел конвертации (`simulator_perception_node` в `console_scripts`).

- `manual_thruster_bridge_node.py`  
  Вспомогательный узел ручного управления тягой (`manual_thruster_bridge_node` в `console_scripts`).

## Структура

- `simulator_perception/`
  - `simulator_perception_node.py`
  - `manual_thruster_bridge_node.py`
  - `utils/DistanceCalculator.py`
  - `utils/__init__.py`
  - `bbox_attrs.yaml`
- `resource/bbox_attrs.yaml`
- `setup.py`
- `package.xml`

## Ресурсы и утилиты

- `bbox_attrs.yaml` — параметры объектов для оценки расстояний/углов.
- `DistanceCalculator` вынесен в `utils/DistanceCalculator.py` и используется в основном узле.

## Entry points

В `setup.py`:
- `simulator_perception_node = simulator_perception.simulator_perception_node:main`
- `manual_thruster_bridge_node = simulator_perception.manual_thruster_bridge_node:main`

## Зависимости

Ключевые зависимости:
- `rclpy`
- `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `vision_msgs`, `std_msgs`, `std_srvs`
- `stingray_interfaces`
- `python3-yaml`
- `ament_index_python`

## Запуск

После сборки и `source install/setup.bash`:

- Основной конвертер:
  - `ros2 run simulator_perception simulator_perception_node`

- Ручной bridge:
  - `ros2 run simulator_perception manual_thruster_bridge_node`