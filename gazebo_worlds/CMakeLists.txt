cmake_minimum_required(VERSION 3.5)
project(gazebo_worlds)

find_package(ros_gz_sim REQUIRED)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~"
  EXCLUDE)

ament_package()
