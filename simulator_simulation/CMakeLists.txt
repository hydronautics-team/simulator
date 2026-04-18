cmake_minimum_required(VERSION 3.5)

project(simulator_simulation)

find_package(ament_cmake REQUIRED)
find_package(simulator_description REQUIRED)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/simulator_simulation.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/simulator_simulation.sh.in")

ament_package()

install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

