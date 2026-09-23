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

Нужно три терминала, все — в одном и том же контейнере:

```bash
docker compose exec simulator bash
```

**Терминал 1 — мир.** Запускаем подводный мир (поверхность на z=0, дно на z=-100):

```bash
ros2 launch gazebo_worlds empty_underwater_world.launch.py
```

**Терминал 2 — робот.** Спавним шар-аппарат (телеоп в этот запуск не входит):

```bash
ros2 launch descriptions upload_rexrov_default.launch.py
```

По умолчанию робот появляется на глубине `z=-20` (камера стартует у поверхности,
поэтому для быстрой проверки удобно `z:=-2.0`), имя — `ball`.

**Терминал 3 — телеоп.** Клавиатурное управление отдельным процессом; команда
читает клавиатуру этого терминала:

```bash
ros2 run descriptions ball_teleop.py --ros-args -p name:=ball
```

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

Параметры телеопа (`max_rpm`, `step_rpm`) задаются через `--ros-args -p`,
например `-p max_rpm:=2000 -p step_rpm:=250`.

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

## Отладка

Вся отладочная обвязка включается **только** аргументом `debug:=true` у спавна:

```bash
ros2 launch descriptions upload_rexrov_default.launch.py \
    debug:=true perspectives:=world_model,odometry
```

Что запускается:

- **Маркеры в сцене** (`gazebo_worlds/scripts/debug_markers.py`): стрелка
  равнодействующей тяги из центра корпуса (длина = сила · `force_scale`,
  по умолчанию 1 мм/Н). Публикуются на `/<name>/debug/marker` и зеркалятся в
  `/marker`, который рендерит встроенный MarkerManager Gazebo
- **Графики** (`gazebo_worlds/scripts/debug_plot.py`, matplotlib): на каждую
  перспективу своё окно, внутри — по subplot'у на кривую.
  `perspectives:=world_model,odometry` открывает оба окна (список — через запятую).
  Окно прокручивается (колесо мыши или Page Up/Down, `q` — закрыть); при большом
  числе кривых (например `odometry` с каналами IMU) subplot'ы раскладываются в две
  колонки и листаются по вертикали
- **Ground truth поза** на `/<name>/debug/pose` (`geometry_msgs/PoseStamped`)

Наборы кривых — YAML-файлы в `descriptions/config/plots/`:

| Набор | Что показывает |
| --- | --- |
| `world_model` | команды и тяги движителей (`/ball/thrusters/...`) |
| `odometry` | мировая поза/ориентация робота (`/ball/debug/pose/...`) + каналы IMU (угловые скорости, ускорения) |
| `mission` | глубина (`/ball/sensors/pressure`), скорость рыскания, позиция |

Формат файла:

```yaml
# {name} подставляется как имя робота
window_title: odometry
window_seconds: 60      # горизонт времени на графике
update_rate: 10         # частота перерисовки, Гц
topics:                 # одна строка = один subplot
  - /{name}/debug/pose/pose/position/x
  - /{name}/sensors/pressure/fluid_pressure
```

Свой набор: скопируй любой файл, поменяй `topics` и запускай
`perspectives:=<имя файла>`. Окна идут через X11 (backend TkAgg); без дисплея
нода работает вхолостую (backend Agg) и только собирает данные.

Просмотр камеры (CV-отладка):

```bash
rqt_image_view /ball/sensors/camera/front
```

Без `debug:=true` ни маркеры, ни окна графиков, ни debug-топики не запускаются.

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
- **Графики идут «столбиками», значения скачут в ноль** — на топик пишут
  несколько издателей: оставшийся телеоп/мост от прошлого запуска или второй
  симулятор в том же ROS-домене (плоттер предупредит в логе: `topic ... has N
  publishers`). Проверь `ros2 node list` и `ros2 topic info -v <топик>`,
  перезапусти спавн/контейнер начисто. Для параллельных запусков используй
  другой `ROS_DOMAIN_ID`, чтобы темы не смешивались.
