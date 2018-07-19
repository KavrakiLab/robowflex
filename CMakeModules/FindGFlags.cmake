find_path(GFLAGS_INCLUDE_DIR NAMES gflags/gflags.h)
find_library(GFLAGS_LIBRARY NAMES gflags)

set(GFLAGS_LIBRARIES ${GFLAGS_LIBRARY})
set(GFLAGS_INCLUDE_DIRS ${GFLAGS_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GFLAGS DEFAULT_MSG
                                  GFLAGS_LIBRARY GFLAGS_INCLUDE_DIR)

mark_as_advanced(GFLAGS_INCLUDE_DIR GFLAGS_LIBRARY)
