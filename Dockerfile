# Базовый образ ROS 2 Iron
FROM ros:iron-ros-base

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg \
    curl \
    build-essential \
    cmake \
    git \
    nano \
    wget \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Добавляем репозиторий Gazebo
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y ignition-fortress ros-iron-ros-gz

# Переменные окружения
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Копируем ВСЕ файлы из текущей папки в /simulator (не создавая вложенности)
COPY . /simulator
WORKDIR /simulator

# Сборка ROS2 пакетов
RUN /bin/bash -c "source /opt/ros/iron/setup.bash && colcon build --packages-select simulator_launch simulator_description simulator_simulation simulator_perception"

# Копируем entrypoint (лежит в папке docker)
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
