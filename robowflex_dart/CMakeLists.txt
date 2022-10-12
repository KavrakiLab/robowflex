cmake_minimum_required(VERSION 3.1...3.10)
project(robowflex_dart
  VERSION 1.3
  LANGUAGES CXX
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
    robowflex_util
  )

  find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES})

  catkin_package(
    LIBRARIES ${LIBRARY_NAME}
    CATKIN_DEPENDS ${CATKIN_SOURCES}
    DEPENDS
    INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

  list(APPEND INCLUDES
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${catkin_INCLUDE_DIRS}
  )

  list(APPEND LIBRARIES ${catkin_LIBRARIES})

  ## External Deps
  list(APPEND INCLUDES ${DART_INCLUDE_DIRS})
  list(APPEND LIBRARIES ${DART_LIBRARIES})

  find_package(ompl REQUIRED)
  if (ompl_LIBRARIES)
    list(APPEND INCLUDES ${ompl_INCLUDE_DIRS})
    list(APPEND LIBRARIES ${ompl_LIBRARIES})
  else()
    list(APPEND INCLUDES ${OMPL_INCLUDE_DIRS})
    list(APPEND LIBRARIES ${OMPL_LIBRARIES})
  endif (ompl_LIBRARIES)

  find_package(TinyXML2 REQUIRED)
  list(APPEND INCLUDES ${TINYXML2_INCLUDE_DIRS})
  list(APPEND LIBRARIES ${TINYXML2_LIBRARIES})


  list(APPEND SOURCES
    src/robot.cpp
    src/io.cpp
    src/space.cpp
    src/world.cpp
    src/acm.cpp
    src/structure.cpp
    src/tsr.cpp
    src/planning.cpp
    src/gui.cpp
    src/joints/joint.cpp
    src/joints/rnjoint.cpp
    src/joints/so2joint.cpp
    src/joints/so3joint.cpp
  )

  include_directories(SYSTEM ${INCLUDES})
  add_library(${LIBRARY_NAME} ${SOURCES})
  set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
  target_link_libraries(${LIBRARY_NAME} PUBLIC ${LIBRARIES})

  add_script(fetch_plan)
  add_script(fetch_ik)
  add_script(fetch_bimanual)
  add_script(fetch_lift)
  add_script(fetch_se2)
  add_script(gui_test)
  add_script(tsr_helper)

  install_scripts()
  install_library()
else()
  message("`DART` not found, not compiling robowflex_dart library or executables.")
endif()
