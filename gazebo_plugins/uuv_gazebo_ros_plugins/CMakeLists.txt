cmake_minimum_required(VERSION 3.5)
project(gazebo_plugins)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(BASE_LIBS
  ament_cmake
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

include_directories(include)

link_directories(${GAZEBO_LIBRARY_DIRS})

# ROS 2 wrapper of the underwater object plugin (buoyancy / hydrodynamics)
add_library(underwater_object_plugin SHARED
  src/UnderwaterObjectROSPlugin.cpp)
ament_target_dependencies(underwater_object_plugin ${BASE_LIBS})

install(TARGETS uuv_underwater_object_ros_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*~"
  EXCLUDE)

ament_export_dependencies(${BASE_LIBS})
ament_export_include_directories(include)
ament_export_libraries(underwater_object_plugin)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(xacro REQUIRED)

  set(GAZEBO_ROS_PLUGINS_PYTHON_TESTS
    test/test_default_fossen_vehicle.test.py
    test/test_sphere_vehicle.test.py
  )

  foreach(T ${GAZEBO_ROS_PLUGINS_PYTHON_TESTS})
    get_filename_component(_test_name ${T} NAME_WE)
    add_launch_test(
      ${T}
      TARGET ${TEST_NAME}
      PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
      APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
      PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 120
    )
  endforeach()
endif()

ament_package()
