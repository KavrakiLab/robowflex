cmake_minimum_required(VERSION 2.8.3)
project(robowflex_movegroup)

set(LIBRARY_NAME ${PROJECT_NAME})

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_ROOT_DIR}/cmake/Modules"
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

include(CompileOptions)
include(HelperFunctions)

find_package(Boost REQUIRED filesystem)
find_library(YAML yaml-cpp REQUIRED)

find_package(catkin QUIET COMPONENTS
  robowflex_library
  actionlib
  moveit_msgs
)

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS
    robowflex_library
    actionlib
    moveit_msgs
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

include_directories(SYSTEM ${CMAKE_CURRENT_LIST_DIR}/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} yaml-cpp)
link_directories(${catkin_LIBRARY_DIRS})

add_library(${LIBRARY_NAME}
    src/services.cpp
  )

set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

add_script(tapedeck)

install_scripts()
install_library()
