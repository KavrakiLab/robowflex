cmake_minimum_required(VERSION 2.8.3)
project(robowflex_library)

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

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  geometric_shapes
  eigen_conversions
  rosbag
  roscpp
  )

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS
    moveit_ros_planning
    geometric_shapes
    eigen_conversions
    rosbag
    roscpp
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
  src/io.cpp
  src/io/visualization.cpp
  src/io/broadcaster.cpp
  src/io/hdf5.cpp
  src/pool.cpp
  src/tf.cpp
  src/yaml.cpp
  src/path.cpp
  src/detail/r2.cpp
  src/detail/ur5.cpp
  src/detail/fetch.cpp
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
## Scripts
##

add_script(ur5_test)
add_script(ur5_cylinder)
add_script(ur5_benchmark)
add_script(ur5_io)
add_script(ur5_pool)
add_script(ur5_visualization)
add_script(fetch_test)
add_script(fetch_scenes_visualize)
add_script(fetch_scenes_benchmark)
add_script(fetch_benchmark)
add_script(fetch_visualization)
add_script(r2_test)
add_script(r2_hdf5)
add_script(wam7_test)
add_script(wam7_benchmark)
add_script(hdf5_io)
add_script(plugin_io)

##
## Installation of programs, library, headers, and YAML used by scripts
##

install_scripts()
install_library()
install_directory(yaml)
