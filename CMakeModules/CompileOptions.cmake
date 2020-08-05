if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if (CMAKE_VERSION VERSION_LESS "3.1")
  add_definitions(-std=c++11)
else()
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  # this next line shouldn't be necessary, but doesn't always get added by cmake (e.g., for clang++-5)
  add_definitions(-std=c++11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-W -Wall -Wextra -Wcast-qual -Wwrite-strings -Wunreachable-code -Wpointer-arith -Winit-self -Wredundant-decls -Wno-unused-parameter -Wno-unused-function -Wno-noexcept-type -Wno-deprecated-declarations)
  # prepend optimizion flag (in case the default setting doesn't include one)
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS_RELEASE}")
endif(CMAKE_COMPILER_IS_GNUCXX)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  add_definitions(-W -Wall -Wextra -Wno-missing-field-initializers -Wno-unused -Wno-unused-parameter -Wno-delete-non-virtual-dtor -Wno-overloaded-virtual -Wno-unknown-pragmas -Qunused-arguments -Wno-deprecated-register -Wno-mismatched-tags -Wno-deprecated-declarations)
  # prepend optimizion flag (in case the default setting doesn't include one)
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS_RELEASE}")
endif()

if((CMAKE_COMPILER_IS_GNUCXX OR IS_ICPC) AND NOT MINGW)
  add_definitions(-fPIC)
endif((CMAKE_COMPILER_IS_GNUCXX OR IS_ICPC) AND NOT MINGW)

# no prefix needed for python modules
set(CMAKE_SHARED_MODULE_PREFIX "")
