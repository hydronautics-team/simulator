# Используем официальный образ ROS2 Humble с desktop (включает множество инструментов, включая Gazebo)
FROM hydronautics/stingray:control

# Обновляем пакеты и устанавливаем необходимые зависимости для Gazebo
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg \
    curl

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y ignition-fortress ros-humble-ros-gz
    
# (Опционально) Устанавливаем переменные окружения для корректной работы X11
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

COPY simulator /simulator
WORKDIR /simulator
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /additional_packages/install/setup.bash && source /stingray_core/install/setup.bash && source /stingray/install/setup.bash && colcon build --packages-select ros_gz_example_bringup ros_gz_example_description ros_gz_example_gazebo py_pub"

# Копируем стандартный ROS2 entrypoint (он уже есть в базовом образе, но можно определить свой)
COPY simulator/docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
