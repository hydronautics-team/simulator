FROM osrf/ros:melodic-desktop-full-bionic

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ADD ./ /catkin_ws

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && cd /catkin_ws && catkin_make"

ENTRYPOINT ["/catkin_ws/entrypoint.sh"]
