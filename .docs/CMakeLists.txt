cmake_minimum_required(VERSION 2.8.3)
project(robowflex_docs)

macro(add_doc project)
  get_filename_component(ABS_FILENAME_SRC "../${project}/src"
    REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

  get_filename_component(ABS_FILENAME_PACKAGE "../${project}/${project}"
    REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

  get_filename_component(ABS_FILENAME_INCLUDE "../${project}/include"
    REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

  get_filename_component(ABS_FILENAME_SCRIPTS "../${project}/scripts"
    REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

  get_filename_component(ABS_FILENAME_README "../${project}/README.md"
    REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

  set(DOC_SOURCES "${DOC_SOURCES} ${ABS_FILENAME_SRC} ${ABS_FILENAME_PACKAGE} ${ABS_FILENAME_INCLUDE} ${ABS_FILENAME_SCRIPTS} ${ABS_FILENAME_README}")
endmacro(add_doc)

add_doc(robowflex_library)
add_doc(robowflex_movegroup)
add_doc(robowflex_ompl)
add_doc(robowflex_tesseract)
add_doc(robowflex_visualization)
add_doc(robowflex_dart)
add_doc(.docker)

find_package(Doxygen REQUIRED)

set(DOXYFILE "${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in")

set(DOXYGEN_SOURCE_DIRS ${DOC_SOURCES})

set(DOXYGEN_DOC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

get_filename_component(DOXYGEN_README "./home.md"
  REALPATH BASE_DIR "${DOXYGEN_DOC_DIR}/markdown/")

set(DOXYGEN_TAG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tag)
set(DOXYGEN_HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/html)
set(DOXYGEN_ROS_VERSION "melodic")

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
# add a target to generate API documentation with Doxygen
add_custom_target(doc ALL
  ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc
  )
