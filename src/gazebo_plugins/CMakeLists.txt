cmake_minimum_required(VERSION 2.8.3)
project(gazebo_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
  add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

find_package(Boost REQUIRED COMPONENTS thread)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include_directories(include) 


# For model_move_plugin plugin
add_library(model_move_plugin src/model_move.cc)
target_link_libraries(model_move_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
                ${Boost_INCLUDE_DIR}
                ${catkin_INCLUDE_DIRS}
                ${GAZEBO_INCLUDE_DIRS}
                ${EIGEN3_INCLUDE_DIRS}
   LIBRARIES model_move
   CATKIN_DEPENDS gazebo_ros roscpp
   DEPENDS system_lib gazebo
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS})

link_directories(${EIGEN3_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS})





#install(DIRECTORY include/
#   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#)


