cmake_minimum_required(VERSION 2.8.3)
project(robowflex_tesseract)

set(LIBRARY_NAME ${PROJECT_NAME})

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_ROOT_DIR}/cmake/Modules"
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

include(CompileOptions)
include(HelperFunctions)

find_package(Boost REQUIRED)

find_package(catkin QUIET COMPONENTS
  ompl
  moveit_planners_ompl
  robowflex_library
  tesseract_planning
  tesseract_ros
)

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS
    ompl
    moveit_planners_ompl
    robowflex_library
    tesseract_planning
    tesseract_ros
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

if (tesseract_planning_DIR)
  include_directories(${CMAKE_CURRENT_LIST_DIR}/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS})

  add_library(${LIBRARY_NAME}
    src/tesseract_planners.cpp
    src/conversions.cpp
    )

  set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
  target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  add_script(tesseract_benchmark)
  add_script(ur5_tesseract)
  add_test(ur_test_env_placement)

  install_scripts()
  install_tests()
  install_library()
else()
  message("`tesseract_planning` not found, not compiling robowflex_tesseract library or executables.")
endif()
