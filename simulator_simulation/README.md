# simulator_simulation

`simulator_simulation` — ROS2-пакет с мирами Gazebo и ресурсами окружения для симулятора.

## Назначение

Пакет:
- хранит SDF-миры и связанные модели окружения;
- устанавливает ресурсы мира в `share/simulator_simulation/worlds`;
- экспортирует environment hooks для корректного поиска ресурсов Gazebo.

## Состав пакета

- `worlds/` — основные миры и наборы моделей:
  - `SAUVC_WORLD.sdf`
  - `man.sdf`
  - `multi_lrauv_race.sdf`
  - `building_robot.sdf`
  - каталоги объектов (`starting_gates`, `qualification_gate`, `pool`, `flare`, `buckets`, и др.)
- `hooks/`
  - `simulator_simulation.dsv.in`
  - `simulator_simulation.sh.in`
- `CMakeLists.txt`
- `package.xml`

## Установка

В `CMakeLists.txt`:

- источник: `worlds/`
- назначение: `share/${PROJECT_NAME}/worlds`

Также подключены environment hooks через:
- `ament_environment_hooks(...simulator_simulation.dsv.in)`
- `ament_environment_hooks(...simulator_simulation.sh.in)`

## Зависимости

Ключевые зависимости из `package.xml`:
- `ament_cmake`
- `simulator_description`

## Использование

Типовой путь использования — через `simulator_launch`:

- основной launch использует мир по умолчанию:
  - `simulator_simulation/worlds/SAUVC_WORLD.sdf`

Пример явного запуска с кастомным миром:

- `ros2 launch simulator_launch sim.launch.py world:=/abs/path/to/world.sdf`

## Добавление нового мира

1. Добавить `.sdf` в `worlds/` или подпапку мира.
2. Если нужны новые объекты, добавить каталог модели с `model.config` и `model.sdf`.
3. Проверить, что ресурсы корректно резолвятся через установленные пути Gazebo.
4. Подключить мир в launch-аргумент `world` при запуске.