# Adds a test (in a source file under the `tests` directory) to the list of
# executables to compile. Adds the name to the list `TESTS`.
macro(add_test_script test_name)
  if(CATKIN_ENABLE_TESTING)
    list(APPEND TESTS test_${test_name})
    catkin_add_gtest(test_${test_name} test/${test_name}.cpp)
    target_link_libraries(test_${test_name} ${LIBRARY_NAME} ${catkin_LIBRARIES})
  endif()
endmacro(add_test_script)

# Install tests added via `add_test` to the install directory.
macro(install_tests)
  install(TARGETS
    ${TESTS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro(install_tests)

# Adds a script (in a source file under the `scripts` directory) to the list of
# executables to compile. Adds the name to the list `SCRIPTS`.
macro(add_script script_name)
  list(APPEND SCRIPTS ${script_name})
  add_executable(${script_name} scripts/${script_name}.cpp)
  target_link_libraries(${script_name} ${LIBRARY_NAME} ${catkin_LIBRARIES})
endmacro(add_script)

# Install scripts added via `add_script` to the install directory.
macro(install_scripts)
  install(TARGETS
    ${SCRIPTS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro(install_scripts)

# Installs library to install directory. Assumes LIBRARY_NAME is set.
macro(install_library)
  install(TARGETS ${LIBRARY_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
endmacro(install_library)

# Installs a directory (e.g., YAML, etc.) to the install directory.
macro(install_directory directory)
  install(DIRECTORY ${directory} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endmacro(install_directory)
