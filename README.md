# simulator

ROS2 + Gazebo (Fortress) симулятор для задач SAUVC.

## 1) Быстрый запуск через Docker (`./db`)

Рекомендуемый способ запуска симулятора — через автоматизированный скрипт `./db`, который сам создаёт и запускает Docker-контейнер с нужным окружением, собирает и стартует симулятор.

### Требования
- Установленный Docker
- Доступ к X11 (для отображения GUI)
- Права на запуск скрипта: `chmod +x ./db`

### Запуск
```bash
./db
```

Скрипт автоматически:
- Проверяет наличие и актуальность контейнера (mount текущего workspace)
- Пересоздаёт контейнер при необходимости
- Собирает только нужные ROS2-пакеты
- Запускает симулятор с основным сценарием

> **Примечание:**
> При первом запуске потребуется загрузка Docker-образа и установка зависимостей (может занять несколько минут).

---

## 2) Актуальная структура проекта

В рабочем состоянии используются пакеты:

- `simulator_launch` — launch-оркестрация запуска
- `simulator_description` — модели и ресурсы робота
- `simulator_simulation` — миры и ресурсы окружения
- `simulator_gazebo_plugins` — C++ плагины Gazebo
- `stingray_interfaces` — контракты `msg/srv/action`
- `simulator_perception` — Python-ноды (включая `converter`)

Пакеты-обертки `simulator_bridge` и `simulator_control` удалены как неиспользуемые.

## 3) Запуск (альтернативный, вручную)

Если требуется ручная сборка и запуск вне Docker:

### Зависимости

- ROS2 Iron
- Gazebo Fortress
- `ros-<distro>-ros-gz`

### Сборка

```bash
colcon build
source install/setup.bash
```

### Основной запуск

```bash
ros2 launch simulator_launch sim.launch.py
```

### Прямой запуск миссии SAUVC

```bash
ros2 launch simulator_launch mission_sauvc.launch.py
```

### Дополнительные launch-файлы

- `simulator_launch/launch/mission_sauvc.launch.py` — основной сценарий SAUVC
- `simulator_launch/launch/world_man.launch.py` — сценарий с world `man`
- `simulator_launch/launch/diff_drive.launch.py` — сценарий diff_drive
- `simulator_launch/launch/rrbot_setup.launch.py` — сценарий rrbot

## 4) Конфигурация

- Bridge-конфиг: `simulator_launch/config/simulator_bridge.yaml`
- RViz-конфиги:
  - `simulator_launch/config/diff_drive.rviz`
  - `simulator_launch/config/rrbot.rviz`

## 5) Контракты взаимодействия (namespace)

- Topics: `/simulator/sensors/*`, `/simulator/perception/*`, `/simulator/control/*`, `/simulator/state/*`
- Services: `/simulator/control/*`, `/simulator/system/*`
- Actions: `/simulator/mission/*`

## 6) Правила изменений

- Все межмодульные контракты оформляются через `stingray_interfaces`.
- Новые runtime-настройки выносятся в YAML.
- Избегать legacy-имен и неинформативных названий файлов.
