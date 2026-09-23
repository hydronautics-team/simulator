# Underwater buoyancy / hydrodynamics simulator workspace.
#
# Base: official OSRF ROS 2 image with the `simulation` variant, which ships
# the Gazebo Sim Noble binaries through the ros_gz vendor packages together
# with ros_gz_sim / ros_gz_bridge / ros_gz_image.
ARG ROS_DISTRO=lyrical
FROM osrf/ros:${ROS_DISTRO}-simulation

# Build, test and GPU (OpenGL) runtime dependencies of this workspace.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libegl1 \
    libeigen3-dev \
    libgl1 \
    libglu1-mesa \
    libxcb-cursor0 \
    mesa-utils \
    python3-colcon-common-extensions \
    python3-matplotlib \
    python3-pytest \
    python3-tk \
    ros-${ROS_DISTRO}-ament-cmake-gtest \
    ros-${ROS_DISTRO}-launch-testing-ament-cmake \
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-rqt-plot \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# NVIDIA container runtime: with the GPU passed through, expose every driver
# capability (the graphics one is required by the Gazebo Sim GUI / GPU
# rendering). Harmless when no GPU is available.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE=/ws

# The workspace sources are copied into the image as a fallback; docker-compose
# mounts the live repo over this directory, and the workspace is built inside
# the container (see ros_entrypoint.sh).
RUN mkdir -p ${WORKSPACE}/src/simulator
COPY . ${WORKSPACE}/src/simulator

WORKDIR ${WORKSPACE}

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Interactive shells started with `docker compose exec ... bash` do not run
# the entrypoint, so the ROS environment is sourced from the shell startup
# file as well.
RUN printf '%s\n' \
    'source /opt/ros/${ROS_DISTRO}/setup.bash' \
    'if [ -f /ws/src/simulator/install/setup.bash ]; then source /ws/src/simulator/install/setup.bash; fi' \
    >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
