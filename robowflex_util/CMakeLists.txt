cmake_minimum_required(VERSION 3.1...3.10)
project(robowflex_util
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

find_package(Eigen3 REQUIRED)

##
## Catkin setup
##

list(APPEND CATKIN_SOURCES
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
  src/random.cpp
  src/pool.cpp
  src/colormap.cpp
  src/gnuplot.cpp
  src/process.cpp
  src/time.cpp
  src/uid.cpp
  )

list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  )

list(APPEND LIBRARIES
  ${catkin_LIBRARIES}
  )

include_directories(SYSTEM ${INCLUDES})
link_directories(${catkin_LIBRARY_DIRS})
add_library(${LIBRARY_NAME} ${SOURCES})
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${LIBRARY_NAME} ${LIBRARIES})

##
## Installation of programs, library, headers, and YAML used by scripts
##

install_library()
