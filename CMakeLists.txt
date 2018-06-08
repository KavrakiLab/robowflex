cmake_minimum_required(VERSION 2.8.3)
project(robowflex)

set(LIBRARY_NAME robowflex)

set(CMAKE_MODULE_PATH
    "${CMAKE_MODULE_PATH}"
    "${CMAKE_ROOT_DIR}/cmake/Modules"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    "${CMAKE_CURRENT_SOURCE_DIR}/robowflex/CMakeModules")

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED filesystem)
find_library(YAML yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  geometric_shapes
  tf2
  roscpp
)

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS
    moveit_ros_planning
    geometric_shapes
    tf2
    roscpp
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} yaml-cpp)
link_directories(${catkin_LIBRARY_DIRS})

add_library(${LIBRARY_NAME} src/planner.cpp src/scene.cpp src/robot.cpp src/geometry.cpp src/util.cpp)
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(planner_test src/main.cpp)
target_link_libraries(planner_test ${LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)
