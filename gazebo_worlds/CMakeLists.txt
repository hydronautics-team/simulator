cmake_minimum_required(VERSION 3.5)
project(gazebo_worlds)

find_package(ros_gz_sim REQUIRED)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

# In-scene debug markers and live plots (started by the spawn launch with
# debug:=true)
install(PROGRAMS scripts/debug_markers.py scripts/debug_plot.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
