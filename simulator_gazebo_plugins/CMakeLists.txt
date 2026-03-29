cmake_minimum_required(VERSION 3.5)

project(simulator_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(simulator_description REQUIRED)

find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-common4 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${ignition-common4_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})

add_library(BasicSystem
  SHARED
  src/BasicSystem.cc
)

target_include_directories(
  BasicSystem PRIVATE include
)

target_link_libraries(BasicSystem PRIVATE
  ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER})

add_library(FullSystem
  SHARED
  src/FullSystem.cc
)

target_include_directories(
  FullSystem PRIVATE include
)

target_link_libraries(FullSystem PRIVATE
  ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER})

install(
  TARGETS BasicSystem FullSystem
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
