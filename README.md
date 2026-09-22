# Simulator

ROS 2 Lyrical + Gazebo Sim Noble (gz-sim 10) workspace для подводного аппарата:
плагины плавучести/гидродинамики/движителей (`gazebo_plugins`), робот-шар с
двумя движителями (`descriptions`), подводные миры (`gazebo_worlds`).

## Что установить на хост

1. **Docker Engine + Compose** — стандартная установка из репозитория Docker.
2. **Драйвер NVIDIA** (`nvidia-smi` уже показывает карту) и
   **NVIDIA Container Toolkit** — он пробрасывает GPU внутрь контейнера:

   ```bash
   curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
     sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

   curl -sL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
     sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
     sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

   sudo apt-get update
   sudo apt-get install -y nvidia-container-toolkit

   sudo nvidia-ctk runtime configure --runtime=docker
   sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
   sudo systemctl restart docker
   ```

   Проверка, что GPU виден Docker'у:

   ```bash
   docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi
   ```

3. **Доступ к X11** (для окна Gazebo): один раз на сессию, либо после перезапуска
   графической сессии:

   ```bash
   xhost +local:root
   ```

## Сборка и вход в контейнер

```bash
docker compose build
docker compose up -d
docker compose exec simulator bash
```

Что уже настроено в `docker-compose.yml` (подробности — в его шапке):

- `gpus: all` + `NVIDIA_DRIVER_CAPABILITIES=all` — GPU и OpenGL внутри контейнера;
- `DISPLAY`, `/tmp/.X11-unix`, `XAUTHORITY` — окно Gazebo на мониторе хоста;
- `network_mode: host` — gz-transport и ROS 2 DDS видят и другие контейнеры, и хост.

Проверка GPU в контейнере:

```bash
nvidia-smi
glxinfo -B | grep -i "OpenGL renderer"   # должно быть NVIDIA GeForce ...
```

## Простейшая миссия: мир → робот → телеоп

Нужно два терминала, оба — в одном и том же контейнере:

```bash
docker compose exec simulator bash
```

**Терминал 1 — мир.** Запускаем подводный мир (поверхность на z=0, дно на z=-100):

```bash
ros2 launch gazebo_worlds empty_underwater_world.launch.py
```

**Терминал 2 — робот и телеоп.** Спавним шар-аппарат и запускаем клавиатурный
телеоп; команда читает клавиатуру этого же терминала:

```bash
docker compose exec simulator bash
ros2 launch descriptions upload_rexrov_default.launch.py
```

По умолчанию робот появляется на глубине `z=-20` (камера стартует у поверхности,
поэтому для быстрой проверки удобно `z:=-2.0`), имя — `ball`.

Управление (`w/a/s/d` или стрелки, пробел — стоп, `q` — выход):

| клавиша | действие |
| --- | --- |
| `w` / `↑` | оба движителя `+step` (вперёд) |
| `s` / `↓` | оба движителя `-step` (назад) |
| `a` / `←` | левый `-step`, правый `+step` (поворот влево) |
| `d` / `→` | левый `+step`, правый `-step` (поворот вправо) |
| `space` | оба движителя в 0 |
| `q` | выход |

Каждое нажатие добавляет `step_rpm` (по умолчанию 100 rpm) к скорости
соответствующих движителей; удержание клавиши добавляет шаги, пока не упрётся в
`max_rpm` (по умолчанию 1500 rpm). Тяга = `rotorConstant * |w| * w`, так что
одинаковые скорости — вперёд, противоположные — вращение.

**Спавн и телеоп по отдельности** (полезно для своих скриптов):

```bash
# только спавн, без телеопа
ros2 launch descriptions upload_rexrov_default.launch.py teleop:=false

# телеоп отдельным процессом
ros2 run descriptions ball_teleop.py --ros-args -p name:=ball
```

Ручная команда движителю (скорость ротора в rad/s, `std_msgs/Float64`):

```bash
ros2 topic pub -r 10 /ball/thrusters/id_0/input std_msgs/msg/Float64 "{data: 300.0}"
```

Встроенный IMU робота (`gz-sim-imu-system`, см. `descriptions/robots/ball.xacro`)
публикуется на gz-топике `/ball/sensors/imu` и бриджится в ROS 2 как
`sensor_msgs/Imu` на 50 Гц:

```bash
ros2 topic echo /ball/sensors/imu --once
```

Передняя камера робота (`front_camera`, 640×480 @ 30 Гц, обзор 60°) идёт в ROS 2
как `sensor_msgs/Image` на `/ball/sensors/camera/front`, калибровка —
`sensor_msgs/CameraInfo` на `/ball/sensors/camera/front/camera_info`:

```bash
ros2 topic echo /ball/sensors/camera/front --once
```

Датчик давления воды (`libwater_pressure.so`) публикует гидростатическое давление
на `/ball/sensors/pressure` как `sensor_msgs/FluidPressure` (10 Гц, ρ = 1028 кг/м³).
Глубина в метрах: `(fluid_pressure − 101325) / (1028 · 9.80665)`:

```bash
ros2 topic echo /ball/sensors/pressure --once
```

## Разработка

Исходники репозитория смонтированы в контейнер как `/ws/src/simulator`; после
правок кода собираем там же:

```bash
cd /ws/src/simulator
colcon build                              # или --packages-select gazebo_worlds descriptions
```

Юнит- и интеграционные тесты:

```bash
colcon test --packages-select gazebo_plugins descriptions
colcon test-result --verbose
```

Архитектура пакетов, устройство плагинов и подводные детали физики описаны в
`AGENTS.md`.

## Частые проблемы

- **`ros2: command not found`** — окружение подхватывается только в новом
  интерактивном шелле; открой `docker compose exec simulator bash` заново.
- **Робот не появился** — мир должен быть уже запущен: `gz model --list` должен
  показывать `sea_floor`, `ocean_surface` (и `ball` после спавна). Если списка
  нет — сначала запусти мир.
- **После правок ничего не изменилось** (`Unable to find uri[model://sun]`,
  предупреждение про `transparency`, старый лонч) — устаревший `install/`:
  пересобери workspace (`colcon build`) в `/ws/src/simulator`.
- **`libEGL warning: ... driver (null)`** — контейнер стартовал без GPU:
  пересоздай его (`docker compose down && docker compose up -d`) и проверь
  `nvidia-smi`; `exec` и `restart` настройки GPU не подхватывают.
- **Окно Gazebo не открывается** — на хосте `xhost +local:root`, проверить
  `echo $DISPLAY` и что смонтирован `/tmp/.X11-unix`.
