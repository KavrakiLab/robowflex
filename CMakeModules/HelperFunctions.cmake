macro(add_test test_name)
  list(APPEND TESTS ${test_name})
  add_executable(${test_name} tests/${test_name}.cpp)
  target_link_libraries(${test_name} ${LIBRARY_NAME} ${catkin_LIBRARIES})
endmacro(add_test)

macro(install_tests)
  install(TARGETS
    ${TESTS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro(install_tests)

macro(add_script script_name)
  list(APPEND SCRIPTS ${script_name})
  add_executable(${script_name} scripts/${script_name}.cpp)
  target_link_libraries(${script_name} ${LIBRARY_NAME} ${catkin_LIBRARIES})
endmacro(add_script)

macro(install_scripts)
  install(TARGETS
    ${SCRIPTS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro(install_scripts)

macro(install_library)
  install(TARGETS ${LIBRARY_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
endmacro(install_library)

macro(install_directory directory)
  install(DIRECTORY ${directory} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endmacro(install_directory)

macro(generate_doxygen)
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
endmacro(generate_doxygen)
