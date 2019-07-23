cmake_minimum_required(VERSION 2.8.3)
project(robowflex_ompl)

set(LIBRARY_NAME ${PROJECT_NAME})

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_ROOT_DIR}/cmake/Modules"
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

include(CompileOptions)
include(HelperFunctions)

find_package(Boost REQUIRED)

find_package(catkin QUIET COMPONENTS
  moveit_planners_ompl
  robowflex_library
)

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS
    moveit_planners_ompl
    robowflex_library
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

if (moveit_planners_ompl_DIR)
  include_directories(SYSTEM ${CMAKE_CURRENT_LIST_DIR}/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS})

  add_library(${LIBRARY_NAME}
    src/ompl_interface.cpp
    )

  set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
  target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  add_script(ur5_ompl_interface)
  add_script(fetch_ompl_interface)

  install_scripts()
  install_library()
else()
  message("`moveit_planners_ompl` not found, not compiling robowflex_ompl library or executables.")
endif()
