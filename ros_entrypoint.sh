#!/bin/bash
set -e

# Проверяем, существует ли файл setup.bash
if [ -f /workspace/simulator/install/setup.bash ]; then
  echo "[INFO] Код уже собран"
  source /workspace/simulator/install/setup.bash
else
  echo "[INFO] Файл install/setup.bash не найден. Запускаем сборку..."
  
  # Источники для ROS2 Iron
  source "/opt/ros/iron/setup.bash"
  
  # Переходим в директорию с проектом
  cd /workspace/simulator
  
  # Сборка пакетов (замени названия на свои)
  if ! colcon build --packages-select simulator_launch simulator_description simulator_simulation simulator_perception; then
    echo "[ERROR] Сборка завершилась с ошибкой. Удаляем build, install, log..."
    rm -rf build install log
    exit 1
  fi
  echo "[INFO] Сборка завершена успешно. Выполняем source install/setup.bash..."
  source /workspace/simulator/install/setup.bash
fi

exec "$@"
