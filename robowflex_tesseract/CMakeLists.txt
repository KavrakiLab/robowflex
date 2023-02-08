cmake_minimum_required(VERSION 3.1...3.10)
project(robowflex_tesseract
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

list(APPEND CATKIN_DEPS
  robowflex_library
  tesseract_planning
  tesseract_ros
  )
 
find_package(catkin QUIET COMPONENTS ${CATKIN_DEPS})

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS ${CATKIN_DEPS}
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

if (tesseract_planning_DIR)
  include_directories(SYSTEM 
  ${CMAKE_CURRENT_LIST_DIR}/include ${catkin_INCLUDE_DIRS})

  link_directories(${catkin_LIBRARY_DIRS})

  add_library(${LIBRARY_NAME}
    src/conversions.cpp
    src/trajopt_planner.cpp
    )

  set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
  target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES})

  add_script(fetch_tabletop_goalpose)
  add_script(fetch_tabletop_goalstate)
  add_script(fetch_tabletop_inits)
  add_script(fetch_tabletop_planning_time)
  add_script(fetch_trajopt)
  add_script(ur5_custom_planning)

  install_scripts()
  install_tests()
  install_library()
else()
  message("`tesseract_planning` not found, not compiling robowflex_tesseract library or executables.")
endif()
