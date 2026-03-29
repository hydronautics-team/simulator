cmake_minimum_required(VERSION 3.5)

project(simulator_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/simulator_description.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/simulator_description.sh.in")

ament_package()
