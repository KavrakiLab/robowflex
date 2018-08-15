cmake_minimum_required(VERSION 2.8.3)
project(tmpack)

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
find_package(GFlags QUIET)

##
## Catkin setup
##

find_package(catkin REQUIRED COMPONENTS
  robowflex_library
  roscpp
  )

if (GFLAGS_FOUND)
  catkin_package(
    LIBRARIES ${LIBRARY_NAME}
    CATKIN_DEPENDS
      robowflex_library
      roscpp
    DEPENDS
    INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
    )

  ##
  ## Library Creation
  ##

  list(APPEND SOURCES
    src/footstep_planner/my_walker.cpp
    )

  list(APPEND INCLUDES
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    yaml-cpp
    )

  list(APPEND LIBRARIES
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
    gflags
    )

  include_directories(${INCLUDES})
  link_directories(${catkin_LIBRARY_DIRS})
  add_library(${LIBRARY_NAME} ${SOURCES})
  set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
  target_link_libraries(${LIBRARY_NAME} ${LIBRARIES})

  ##
  ## Scripts
  ##

  list(APPEND SCRIPTS toy_planner)
  add_executable(toy_planner src/main.cpp)
  target_link_libraries(toy_planner ${LIBRARY_NAME} ${catkin_LIBRARIES})

  ##
  ## Install Files
  ##

  install(TARGETS
      ${SCRIPTS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install_library()
  install(DIRECTORY
      src/footstep_planner/scenes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scenes)

else()
  catkin_package(
    LIBRARIES
    CATKIN_DEPENDS
      robowflex_library
      roscpp
    DEPENDS
    INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
    )

  message("`gflags` not found, not compiling TMPack executables.")
endif()
