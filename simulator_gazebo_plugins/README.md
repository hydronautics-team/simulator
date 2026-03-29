# simulator_gazebo_plugins

Пакет `simulator_gazebo_plugins` содержит C++ плагины для Gazebo (Ignition/Gazebo Sim), используемые в симуляторе.

## Назначение

Пакет отвечает за:
- сборку и установку shared library-плагинов Gazebo;
- подключение системной логики симуляции через плагины;
- интеграцию с остальными пакетами стека.

## Состав пакета

- `include/simulator_gazebo_plugins/`
  - `BasicSystem.hh`
  - `FullSystem.hh`
- `src/`
  - `BasicSystem.cc`
  - `FullSystem.cc`
- `CMakeLists.txt`
- `package.xml`

## Зависимости

Основные зависимости из `CMakeLists.txt` и `package.xml`:

- `ament_cmake`
- `simulator_description`
- `ignition-cmake2`
- `ignition-plugin1`
- `ignition-common4`
- `ignition-gazebo6`

## Сборка и установка

Собираются две библиотеки:

- `BasicSystem` (из `src/BasicSystem.cc`)
- `FullSystem` (из `src/FullSystem.cc`)

Установка выполняется в:

- `lib/${PROJECT_NAME}`

То есть после сборки библиотеки доступны в:
- `install/simulator_gazebo_plugins/lib/simulator_gazebo_plugins/`

## Использование в симуляции

Плагины подключаются из SDF/миров или связанных launch-сценариев, где указывается имя плагина и путь к установленной библиотеке.

Типовой сценарий:
1. Собрать workspace (`colcon build`).
2. Подключить окружение (`source install/setup.bash`).
3. Запустить launch из `simulator_launch`, который поднимает Gazebo с нужным world/SDF.

## Добавление нового плагина

1. Добавить заголовок в `include/simulator_gazebo_plugins/`.
2. Добавить реализацию в `src/`.
3. Объявить новую `add_library(... SHARED ...)` в `CMakeLists.txt`.
4. Добавить `target_include_directories(...)` и `target_link_libraries(...)`.
5. Добавить цель в блок `install(TARGETS ...)`.