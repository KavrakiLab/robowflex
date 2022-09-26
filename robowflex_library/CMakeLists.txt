cmake_minimum_required(VERSION 3.1...3.10)
project(robowflex_library
  VERSION 1.3
  LANGUAGES C CXX
)

set(LIBRARY_NAME ${PROJECT_NAME})

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_ROOT_DIR}/cmake/Modules"
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

include(CompileOptions)
include(HelperFunctions)

##
## Non-ROS libraries
##

find_package(Boost REQUIRED filesystem)
find_library(YAML yaml-cpp REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX)

##
## Catkin setup
##

list(APPEND CATKIN_SOURCES
  robowflex_util

  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  shape_msgs
  octomap_msgs
  trajectory_msgs
  object_recognition_msgs
  moveit_msgs

  actionlib
  roscpp
  rosbag
  urdf
  srdfdom
  pluginlib
  geometric_shapes
  moveit_core
  moveit_ros_planning
  moveit_planners_ompl
  tf2_ros
  xmlrpcpp
  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES})

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS ${CATKIN_SOURCES}
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

##
## Library Creation
##

list(APPEND SOURCES
  src/planning.cpp
  src/builder.cpp
  src/scene.cpp
  src/robot.cpp
  src/geometry.cpp
  src/benchmarking.cpp
  src/util.cpp
  src/id.cpp
  src/io.cpp
  src/log.cpp
  src/openrave.cpp
  src/io/visualization.cpp
  src/io/colormap.cpp
  src/io/broadcaster.cpp
  src/io/hdf5.cpp
  src/io/gnuplot.cpp
  src/pool.cpp
  src/tf.cpp
  src/random.cpp
  src/yaml.cpp
  src/trajectory.cpp
  src/ompl_interface.cpp
  src/ompl_trajectory.cpp
  src/movegroup.cpp
  src/detail/ur5.cpp
  src/detail/fetch.cpp
  src/detail/cob4.cpp
  )

list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${TINYXML2_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
  yaml-cpp
  )

list(APPEND LIBRARIES
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TINYXML2_LIBRARIES}
  ${HDF5_LIBRARIES}
  yaml-cpp
  )

include_directories(SYSTEM ${INCLUDES})
link_directories(${catkin_LIBRARY_DIRS})
add_library(${LIBRARY_NAME} ${SOURCES})
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${LIBRARY_NAME} ${LIBRARIES})

##
## Tests
##

add_test_script(robot_scene)
add_test_script(yaml)

##
## Installation of programs, library, headers, and YAML used by scripts
##

install_scripts()
install_library()
install_directory(yaml)
