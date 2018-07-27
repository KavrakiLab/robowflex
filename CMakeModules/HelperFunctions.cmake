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

macro(add_doc project)
  get_filename_component(ABS_FILENAME_SRC "../${project}/src"
    REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

  get_filename_component(ABS_FILENAME_INCLUDE "../${project}/include"
    REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

  get_filename_component(ABS_FILENAME_SRC "../${project}/README.md"
    REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

  set(DOC_SOURCES "${DOC_SOURCES} ${ABS_FILENAME_SRC} ${ABS_FILENAME_INCLUDE}")
endmacro(add_doc)
