cmake_minimum_required(VERSION 2.8.3)
project(robowflex_library)

set(LIBRARY_NAME robowflex_library)

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_ROOT_DIR}/cmake/Modules"
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
  "${CMAKE_CURRENT_SOURCE_DIR}/robowflex/CMakeModules")

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

##
## Non-ROS libraries
##

find_package(Boost REQUIRED filesystem)
find_library(YAML yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

##
## Catkin setup
##

find_package(catkin REQUIRED COMPONENTS
  # moveit_planners_ompl
  moveit_ros_planning
  geometric_shapes
  eigen_conversions
  rosbag
  roscpp
  )

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS
    # moveit_planners_ompl
    moveit_ros_planning
    geometric_shapes
    eigen_conversions
    rosbag
    roscpp
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

##
## Doxygen generation
##

find_package(Doxygen QUIET)
if (DOXYGEN_FOUND)
  set(DOXYFILE "${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in")
  set(DOXYGEN_SOURCE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include")
  set(DOXYGEN_DOC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  set(DOXYGEN_TAG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/tag)
  set(DOXYGEN_HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  set(DOXYGEN_CATKIN_DOC_DIR "${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/doc")

  file(MAKE_DIRECTORY ${DOXYGEN_CATKIN_DOC_DIR})

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  # add a target to generate API documentation with Doxygen
  add_custom_target(doc ALL
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${DOXYGEN_CATKIN_DOC_DIR}
    )

  install(DIRECTORY ${DOXYGEN_CATKIN_DOC_DIR} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
  message("Unable to find Doxygen. API Documentation will not be generated")
endif()

##
## Library Creation
##

include_directories(${CMAKE_CURRENT_LIST_DIR}/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} yaml-cpp)
link_directories(${catkin_LIBRARY_DIRS})

add_library(${LIBRARY_NAME}
  src/planning.cpp
  src/builder.cpp
  src/scene.cpp
  src/robot.cpp
  src/geometry.cpp
  src/benchmarking.cpp
  src/util.cpp
  src/io.cpp
  src/io/visualization.cpp
  src/pool.cpp
  src/tf.cpp
  src/yaml.cpp
  src/detail/r2.cpp
  src/detail/ur5.cpp
  )

set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

##
## Scripts
##

add_executable(ur5_test scripts/ur5_test.cpp)
target_link_libraries(ur5_test ${LIBRARY_NAME} ${catkin_LIBRARIES})

add_executable(ur5_benchmark scripts/ur5_benchmark.cpp)
target_link_libraries(ur5_benchmark ${LIBRARY_NAME} ${catkin_LIBRARIES})

add_executable(ur5_io scripts/ur5_io.cpp)
target_link_libraries(ur5_io ${LIBRARY_NAME} ${catkin_LIBRARIES})

add_executable(ur5_pool scripts/ur5_pool.cpp)
target_link_libraries(ur5_pool ${LIBRARY_NAME} ${catkin_LIBRARIES})

add_executable(ur5_visualization scripts/ur5_visualization.cpp)
target_link_libraries(ur5_visualization ${LIBRARY_NAME} ${catkin_LIBRARIES})

add_executable(r2_test scripts/r2_test.cpp)
target_link_libraries(r2_test ${LIBRARY_NAME} ${catkin_LIBRARIES})

add_executable(wam7_test scripts/wam7_test.cpp)
target_link_libraries(wam7_test ${LIBRARY_NAME} ${catkin_LIBRARIES})

add_executable(wam7_benchmark scripts/wam7_benchmark.cpp)
target_link_libraries(wam7_benchmark ${LIBRARY_NAME} ${catkin_LIBRARIES})

add_executable(tapedeck scripts/tapedeck.cpp)
target_link_libraries(tapedeck ${LIBRARY_NAME} ${catkin_LIBRARIES})

##
## Installation of programs, library, headers, and YAML used by scripts
##

install(TARGETS
    ur5_test
    ur5_benchmark
    ur5_io
    ur5_pool
    ur5_visualization
    r2_test
    wam7_test
    wam7_benchmark
    tapedeck
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${LIBRARY_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/robowflex_library/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
