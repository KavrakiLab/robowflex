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

find_package(Boost REQUIRED filesystem system)
find_package(Eigen3 REQUIRED)

execute_process(
  COMMAND rosversion -s roscpp
  COMMAND cut -d "." -f 2 -s
  OUTPUT_VARIABLE ROS_VERSION_NUMBER
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

configure_file(
  ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/macros.h.in
  ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/macros.h
)

##
## Catkin setup
##

if (CATKIN_DEVEL_PREFIX)
  find_package(catkin REQUIRED)

  catkin_package(
    LIBRARIES ${LIBRARY_NAME}
    DEPENDS boost eigen3
    INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )
endif()

##
## Library Creation
##

list(APPEND SOURCES
  src/filesystem.cpp
  src/random.cpp
  src/pool.cpp
  src/colormap.cpp
  src/gnuplot.cpp
  src/process.cpp
  src/time.cpp
  src/uid.cpp
  src/math.cpp
  src/geometry.cpp
  src/log.cpp
  )

list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

list(APPEND LIBRARIES
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

include_directories(SYSTEM ${INCLUDES})
add_library(${LIBRARY_NAME} ${SOURCES})
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${LIBRARY_NAME} PUBLIC ${LIBRARIES})

if (CATKIN_DEVEL_PREFIX)
  install_library()
endif()
