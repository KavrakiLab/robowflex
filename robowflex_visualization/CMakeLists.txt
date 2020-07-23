cmake_minimum_required(VERSION 2.8.3)
project(robowflex_visualization)

find_package(catkin REQUIRED COMPONENTS
  robowflex_library
  collada_urdf
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES robowflex_visualization
  CATKIN_DEPENDS 
    robowflex_library
    collada_urdf
  DEPENDS
)

install(PROGRAMS
  scripts/robowflex.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)