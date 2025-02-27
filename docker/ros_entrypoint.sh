#!/bin/bash
set -e

# Источник настроек ROS2
source /simulator/install/setup.bash

# Запускаем команду, переданную контейнеру
exec "$@"
