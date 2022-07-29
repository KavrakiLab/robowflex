find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h)
find_library(TINYXML2_LIBRARIES NAMES tinyxml2)

set(TINYXML2_LIBRARIES ${TINYXML2_LIBRARY})
set(TINYXML2_INCLUDE_DIRS ${TINYXML2_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TinyXML2 DEFAULT_MSG
                                  TINYXML2_LIBRARIES TINYXML2_INCLUDE_DIR)

mark_as_advanced(TINYXML2_INCLUDE_DIR TINYXML2_LIBRARIES)
