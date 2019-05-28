cmake_minimum_required(VERSION 2.8.3)
project(robowflex_tesseract)

set(LIBRARY_NAME ${PROJECT_NAME})

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_ROOT_DIR}/cmake/Modules"
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

include(CompilerSettings)
include(HelperFunctions)

find_package(Boost REQUIRED)

list(APPEND CATKIN_DEPS
  ompl
  moveit_planners_ompl
  robowflex_library
  tesseract_planning
  tesseract_ros
  )

find_package(catkin QUIET COMPONENTS ${CATKIN_DEPS})

if (ompl_DIR MATCHES "/usr*")
  message("OMPL not found via catkin. Looking elsewhere...")
  list(REMOVE_AT CATKIN_DEPS 0)
  find_package(ompl REQUIRED)
  set(OMPL_INCS ${OMPL_INCLUDE_DIRS})
  set(OMPL_LIBS ${OMPL_LIBRARY_DIRS})
else()
  set(OMPL_INCS "")
  set(OMPL_LIBS "")
endif()

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS ${CATKIN_DEPS}
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

if (tesseract_planning_DIR)
  include_directories(${CMAKE_CURRENT_LIST_DIR}/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS} ${OMPL_LIBS})

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
