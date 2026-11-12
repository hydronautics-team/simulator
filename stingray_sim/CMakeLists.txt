cmake_minimum_required(VERSION 3.8)
project(stingray_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/state_adapter.py
  scripts/control_adapter.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
