cmake_minimum_required(VERSION 2.8.3)
project(robowflex_movegroup)

# set(LIBRARY_NAME ${PROJECT_NAME})

# add_compile_options(-std=c++11)

# if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Release)
# endif()

# find_package(Boost REQUIRED)

# find_package(catkin QUIET COMPONENTS
#   robowflex_library
# )

# catkin_package(
#   LIBRARIES ${LIBRARY_NAME}
#   CATKIN_DEPENDS
#     robowflex_library
#   DEPENDS
#   INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
#   )

# include_directories(${CMAKE_CURRENT_LIST_DIR}/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
# link_directories(${catkin_LIBRARY_DIRS})

# add_library(${LIBRARY_NAME}
#   )

# set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
# target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_executable(tesseract_interface scripts/tesseract_benchmark.cpp)
# target_link_libraries(tesseract_interface ${LIBRARY_NAME} ${catkin_LIBRARIES})

# add_executable(ur5_tesseract scripts/ur5_tesseract.cpp)
# target_link_libraries(ur5_tesseract ${LIBRARY_NAME} ${catkin_LIBRARIES})

# add_executable(ur_env_test test/ur_test_env_placement.cpp)
# target_link_libraries(ur_env_test ${LIBRARY_NAME} ${catkin_LIBRARIES})

# install(TARGETS ${LIBRARY_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(DIRECTORY include/robowflex_tesseract/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
