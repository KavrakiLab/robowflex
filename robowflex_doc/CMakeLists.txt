cmake_minimum_required(VERSION 2.8.3)
project(robowflex_doc)

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_ROOT_DIR}/cmake/Modules"
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

include(HelperFunctions)
find_package(catkin REQUIRED)

catkin_package()

add_doc(robowflex_library)
add_doc(robowflex_movegroup)
add_doc(robowflex_ompl)
add_doc(robowflex_tesseract)
add_doc(robowflex_visualization)

find_package(Doxygen QUIET)
if (DOXYGEN_FOUND)
  set(DOXYFILE "${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in")

  set(DOXYGEN_SOURCE_DIRS ${DOC_SOURCES})

  get_filename_component(DOXYGEN_README "../README.md"
    REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

  set(DOXYGEN_DOC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  set(DOXYGEN_TAG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/tag)
  set(DOXYGEN_HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  set(DOXYGEN_CATKIN_DOC_DIR "${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/doc")
  set(DOXYGEN_ROS_VERSION $ENV{ROS_DISTRO})

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
