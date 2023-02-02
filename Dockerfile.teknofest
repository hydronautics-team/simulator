FROM hydronautics/uuv_simulator:noetic

ADD ./ /simulator

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && source /uuv_simulator/devel/setup.bash && cd /simulator && catkin_make"

ENTRYPOINT ["/simulator/entrypoint/teknofest.sh"]