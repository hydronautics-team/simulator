# Используем официальный образ ROS2 Humble с desktop (включает множество инструментов, включая Gazebo)
FROM hydronautics/stingray:control

# Обновляем пакеты и устанавливаем необходимые зависимости для Gazebo
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg \
    curl \
    git \
    python3-pip

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y ignition-fortress ros-humble-ros-gz

# Установка rosbridge из исходников
RUN git clone https://github.com/RobotWebTools/rosbridge_suite.git /tmp/rosbridge_suite && \
    cd /tmp/rosbridge_suite && \
    sed -i 's/ament_cmake_mypy//g' rosbridge_library/CMakeLists.txt && \
    sed -i '/ament_mypy/d' rosbridge_library/CMakeLists.txt && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DBUILD_TESTING=OFF" && \
    pip install cbor2

# (Опционально) Устанавливаем переменные окружения для корректной работы X11
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

COPY simulator /simulator
WORKDIR /simulator
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /additional_packages/install/setup.bash && source /stingray_core/install/setup.bash && source /stingray/install/setup.bash && colcon build --packages-select simulator_launch simulator_description simulator_simulation simulator_perception"

# Копируем стандартный ROS2 entrypoint
COPY simulator/docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Добавляем source rosbridge в bashrc
RUN echo 'source /tmp/rosbridge_suite/install/setup.bash' >> /root/.bashrc

# Открываем порт для rosbridge WebSocket
EXPOSE 9090

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
