#!/bin/bash
set -e

# Проверяем, существует ли файл setup.bash
if [ -f /simulator/install/setup.bash ]; then
  echo "[INFO] Код уже сбилжен"
  source /simulator/install/setup.bash
else
  echo "[INFO] Файл install/setup.bash не найден. Запускаем сборку..."
  # Выполняем сборку, при ошибке удаляем build, install, log и выходим

  source "/opt/ros/humble/setup.bash"
  source "/additional_packages/install/setup.bash"
  source /stingray_core/install/setup.bash
  source /stingray/install/setup.bash

  if ! colcon build --packages-select simulator_launch simulator_description simulator_simulation simulator_perception; then
    echo "[ERROR] Сборка завершилась с ошибкой. Удаляем build, install, log..."
    rm -rf build install log
    exit 1
  fi
  echo "[INFO] Сборка завершена успешно. Выполняем source install/setup.bash..."
  source /simulator/install/setup.bash
fi

exec "$@"