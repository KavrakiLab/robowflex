cmake_minimum_required(VERSION 3.1...3.10)
project(robowflex_moveit_scripts
  VERSION 1.3
  LANGUAGES C CXX
)

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_ROOT_DIR}/cmake/Modules"
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

include(CompileOptions)
include(HelperFunctions)

##
## Catkin setup
##

list(APPEND CATKIN_SOURCES
  robowflex_moveit
  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES})

catkin_package(
  CATKIN_DEPENDS ${CATKIN_SOURCES}
)


list(APPEND INCLUDES
  ${catkin_INCLUDE_DIRS}
)

list(APPEND LIBRARIES
  ${catkin_LIBRARIES}
)

include_directories(SYSTEM ${INCLUDES})
link_directories(${catkin_LIBRARY_DIRS})

##
## Scripts
##

add_script(ur5_test)
add_script(ur5_cylinder)
add_script(ur5_benchmark)
add_script(ur5_io)
add_script(ur5_pool)
add_script(ur5_visualization)
add_script(ur5_ik)
add_script(ur5_cartesian)
add_script(fetch_test)
add_script(fetch_chomp)
add_script(fetch_scenes_visualize)
add_script(fetch_scenes_benchmark)
add_script(fetch_benchmark)
add_script(fetch_visualization)
add_script(fetch_cartesian)
add_script(wam7_test)
add_script(wam7_benchmark)
add_script(resources_visualization)
add_script(plugin_io)
add_script(baxter_multi_target)
add_script(shadowhand_ik)
add_script(cob4_test)
add_script(cob4_visualization)
add_script(cob4_multi_target)

add_script(ur5_ompl_interface)
add_script(fetch_ompl_interface)
add_script(fetch_ompl_benchmark)
add_script(fetch_ompl_scenes_benchmark)
add_script(fetch_profile)

add_script(movegroup_record)

##
## Installation of programs, library, headers, and YAML used by scripts
##

install_scripts()
