cmake_minimum_required(VERSION 3.1...3.10)
project(robowflex_ompl
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

find_package(Boost REQUIRED COMPONENTS serialization)

find_package(catkin QUIET COMPONENTS
  robowflex_library
  moveit_planners_ompl
)

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS
    robowflex_library
    moveit_planners_ompl
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

if (moveit_planners_ompl_DIR)
  include_directories(SYSTEM ${CMAKE_CURRENT_LIST_DIR}/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS})

  add_library(${LIBRARY_NAME}
    src/ompl_interface.cpp
    src/ompl_trajectory.cpp
    )

  set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
  target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  add_script(ur5_ompl_interface)
  add_script(fetch_ompl_interface)
  add_script(fetch_ompl_benchmark)
  add_script(fetch_ompl_scenes_benchmark)
  add_script(fetch_profile)

  install_scripts()
  install_library()
else()
  message("`moveit_planners_ompl` not found, not compiling robowflex_ompl library or executables.")
endif()
