cmake_minimum_required(VERSION 2.8.3)
project(tmpack)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED filesystem)
find_library(YAML yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  robowflex_library
  roscpp
)

find_package(gflags QUIET)

catkin_package(
  CATKIN_DEPENDS
   robowflex_library
   roscpp
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} yaml-cpp)
link_directories(${catkin_LIBRARY_DIRS})

if(gflags_FOUND)
  add_executable(toy_planner src/main.cpp)
  target_link_libraries(toy_planner ${LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp gflags)
endif()
