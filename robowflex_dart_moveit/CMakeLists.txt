cmake_minimum_required(VERSION 3.1...3.10)
project(robowflex_dart_moveit
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


find_package(DART 6 QUIET
  COMPONENTS utils utils-urdf gui-osg)
if (DART_LIBRARIES)

list(APPEND CATKIN_SOURCES
    robowflex_moveit
    robowflex_dart
    )

  find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES})

  catkin_package(
    LIBRARIES ${LIBRARY_NAME}
    CATKIN_DEPENDS ${CATKIN_SOURCES}
    DEPENDS
    INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

  list(APPEND INCLUDES ${DART_INCLUDE_DIRS})
  list(APPEND LIBRARIES ${DART_LIBRARIES})

  list(APPEND INCLUDES
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${catkin_INCLUDE_DIRS}
  )

  list(APPEND LIBRARIES
    ${catkin_LIBRARIES}
  )

  list(APPEND SOURCES
    src/dart_planner.cpp
    src/planning_conversions.cpp
    src/robot_conversions.cpp
    src/scene_conversions.cpp
  )

  include_directories(SYSTEM ${INCLUDES})
  link_directories(${catkin_LIBRARY_DIRS})
  add_library(${LIBRARY_NAME} ${SOURCES})
  set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
  target_link_libraries(${LIBRARY_NAME} PUBLIC ${LIBRARIES})

  ##
  ## Scripts
  ##

  add_script(fetch_dart_planner)
  add_script(fetch_dart_moveit_pick)

  ##
  ## Installation of programs, library, headers, and YAML used by scripts
  ##

  install_scripts()
  install_library()
else()
  message("`DART` not found, not compiling robowflex_dart_moveit library or executables.")
endif()
