# simulator_launch

Пакет `simulator_launch` — точка входа для запуска стека симуляции.

## Назначение

Пакет содержит launch-сценарии и конфигурацию, которые:
- запускают Gazebo Sim;
- поднимают мост `ros_gz_bridge`;
- запускают узлы адаптации/перцепции для работы стека.

## Актуальные launch-файлы

В пакете используются:

- `launch/sim.launch.py`  
  Основной входной launch. Объявляет аргументы и включает миссионный launch.

- `launch/mission_sauvc.launch.py`  
  Запускает:
  - Gazebo (`ros_gz_sim`),
  - мост топиков (`ros_gz_bridge parameter_bridge`),
  - узел `simulator_perception/converter`.

## Параметры запуска

Основные аргументы:

- `world` — путь к SDF миру (по умолчанию `simulator_simulation/worlds/SAUVC_WORLD.sdf`);
- `use_sim_time` — использовать симуляционное время (`true` по умолчанию).

## Установка ресурсов

В `CMakeLists.txt` устанавливаются каталоги:

- `launch` → `share/simulator_launch/launch`
- `config` → `share/simulator_launch/config`

## Зависимости

Ключевые зависимости пакета:

- `ros_gz`
- `ros_gz_sim`
- `simulator_description`
- `simulator_simulation`
- `simulator_gazebo_plugins`
- `simulator_perception`

## Запуск

Пример запуска:

- `ros2 launch simulator_launch sim.launch.py`

Пример с явным миром:

- `ros2 launch simulator_launch sim.launch.py world:=/abs/path/to/world.sdf`