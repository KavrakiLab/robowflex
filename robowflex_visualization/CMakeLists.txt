cmake_minimum_required(VERSION 3.1...3.10)
project(robowflex_visualization
  VERSION 1.3
)

find_package(catkin QUIET COMPONENTS
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

if (collada_urdf_DIR)
  install(PROGRAMS
    scripts/robowflex.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  message("`collada_urdf` not found, not compiling robowflex_visualization library or executables.")
endif()
