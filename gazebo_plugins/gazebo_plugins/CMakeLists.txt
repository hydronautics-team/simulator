cmake_minimum_required(VERSION 3.5)
project(gazebo_plugins)

find_package(ament_cmake REQUIRED)
# Gazebo Sim Noble, vendored by the ros_gz stack: the vendor packages export
# unversioned CMake packages (gz-sim, gz-msgs, gz-transport, sdformat).
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(sdformat_vendor REQUIRED)
find_package(sdformat REQUIRED)
find_package(Eigen3 REQUIRED)

# ROS 2

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---- Models (physics) -------------------------------------------------------
add_library(buoyancy_model STATIC src/BuoyancyModel.cpp)
target_include_directories(buoyancy_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>)
target_compile_features(buoyancy_model PUBLIC cxx_std_17)
target_link_libraries(buoyancy_model PUBLIC Eigen3::Eigen)
set_target_properties(buoyancy_model PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(hydrodynamic_model STATIC src/HydrodynamicModel.cpp)
target_include_directories(hydrodynamic_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>)
target_compile_features(hydrodynamic_model PUBLIC cxx_std_17)
target_link_libraries(hydrodynamic_model PUBLIC Eigen3::Eigen)
set_target_properties(hydrodynamic_model PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

# Hydrostatic pressure of the water column (water pressure / depth sensor).
add_library(water_pressure_model STATIC src/WaterPressureModel.cpp)
target_include_directories(water_pressure_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>)
target_compile_features(water_pressure_model PUBLIC cxx_std_17)
set_target_properties(water_pressure_model PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

# Propeller dynamics and thrust conversion. Both are OBJECT libraries: their
# models register themselves with their factory through a static initializer,
# so the objects must always end up inside the binary that uses the factory
# (linking them into the plugins) instead of living in their own shared
# library.
add_library(propeller OBJECT
  src/ZeroOrderPropeller.cpp
  src/PropellerDynamicsFactory.cpp)
target_include_directories(propeller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>)
target_compile_features(propeller PUBLIC cxx_std_17)
target_link_libraries(propeller PUBLIC sdformat::core)
set_target_properties(propeller PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(thruster_converter OBJECT
  src/BasicThrusterConverter.cpp
  src/ThrusterConverterFactory.cpp)
target_include_directories(thruster_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>)
target_compile_features(thruster_converter PUBLIC cxx_std_17)
target_link_libraries(thruster_converter PUBLIC sdformat::core)
set_target_properties(thruster_converter PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

# NOTE: the former standalone plugin (formerly libunderwater_object_system.so)
# is superseded by the UnderwaterObjectPlugin facade below and is not built
# anymore; its source is kept in legacy_classic/ for reference only.

# Facade plugin: buoyancy + hydrodynamics (gz sim), unified name
# "underwater_object" (library libunderwater_object.so)
add_library(underwater_object SHARED
  src/UnderwaterObjectPlugin.cpp)
target_compile_features(underwater_object PRIVATE cxx_std_17)
target_include_directories(underwater_object PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(underwater_object PRIVATE
  buoyancy_model
  hydrodynamic_model
  gz-sim::core
  gz-transport::core
  gz-msgs::core)

# Thruster facade: propeller dynamics + thrust conversion (gz sim), plugin
# name "thruster" (library libthruster.so)
add_library(thruster SHARED
  src/ThrusterPlugin.cpp)
target_compile_features(thruster PRIVATE cxx_std_17)
target_include_directories(thruster PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(thruster PRIVATE
  propeller
  thruster_converter
  sdformat::core
  Eigen3::Eigen
  gz-sim::core
  gz-transport::core
  gz-msgs::core)

# Water pressure facade: hydrostatic pressure of the water column (gz sim
# system plugin name "water_pressure", library libwater_pressure.so)
add_library(water_pressure SHARED
  src/WaterPressurePlugin.cpp)
target_compile_features(water_pressure PRIVATE cxx_std_17)
target_include_directories(water_pressure PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(water_pressure PRIVATE
  water_pressure_model
  gz-sim::core
  gz-transport::core
  gz-msgs::core)

# The plugin and its model libraries share the same install location, so the
# objects are linked straight into the plugins.
install(TARGETS underwater_object thruster water_pressure
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(gz_sim_vendor)
ament_export_dependencies(gz_msgs_vendor)
ament_export_dependencies(gz_transport_vendor)
ament_export_dependencies(sdformat_vendor)
ament_export_libraries(underwater_object thruster water_pressure)

# Make the spawned world find the system plugins after `source install/setup.bash`
# (settles GZ_SIM_SYSTEM_PLUGIN_PATH / IGN_GAZEBO_SYSTEM_PLUGIN_PATH to lib/).
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugins.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_buoyancy_model test/test_buoyancy_model.cpp)
  target_link_libraries(test_buoyancy_model buoyancy_model Eigen3::Eigen)
  target_compile_features(test_buoyancy_model PRIVATE cxx_std_17)

  ament_add_gtest(test_hydrodynamic_model test/test_hydrodynamic_model.cpp)
  target_link_libraries(test_hydrodynamic_model hydrodynamic_model Eigen3::Eigen)
  target_compile_features(test_hydrodynamic_model PRIVATE cxx_std_17)

  ament_add_gtest(test_propeller test/test_propeller.cpp)
  target_link_libraries(test_propeller propeller)
  target_compile_features(test_propeller PRIVATE cxx_std_17)

  ament_add_gtest(test_thruster_converter test/test_thruster_converter.cpp)
  target_link_libraries(test_thruster_converter thruster_converter)
  target_compile_features(test_thruster_converter PRIVATE cxx_std_17)

  ament_add_gtest(test_water_pressure_model test/test_water_pressure_model.cpp)
  target_link_libraries(test_water_pressure_model water_pressure_model)
  target_compile_features(test_water_pressure_model PRIVATE cxx_std_17)

  add_launch_test(test/test_underwater_object_integration.test.py
    TIMEOUT 300)

  add_launch_test(test/test_thruster_integration.test.py
    TIMEOUT 300)
endif()

ament_package()
