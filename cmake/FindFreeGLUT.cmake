# - Try to find FREEGLUT
#
# The following variables are optionally searched for defaults
#  FREEGLUT_ROOT_DIR:            Base directory where all FREEGLUT components are found
#
# The following are set after configuration is done:
#  FREEGLUT_FOUND
#  FREEGLUT_INCLUDE_DIRS
#  FREEGLUT_LIBRARIES
#  FREEGLUT_LIBRARYRARY_DIRS

include(FindPackageHandleStandardArgs)

set(FREEGLUT_ROOT_DIR "" CACHE PATH "Folder contains FreeGLUT")

# We are testing only a couple of files in the include directories
if(WIN32)
    find_path(FREEGLUT_INCLUDE_DIR GL/freeglut.h
        PATHS ${FREEGLUT_ROOT_DIR}/src/windows)
else()
    find_path(FREEGLUT_INCLUDE_DIR GL/freeglut.h
        PATHS ${FREEGLUT_ROOT_DIR})
endif()

if(MSVC)
    find_library(FREEGLUT_LIBRARY_RELEASE
        NAMES libglut
        PATHS ${FREEGLUT_ROOT_DIR}
        PATH_SUFFIXES Release)

    find_library(FREEGLUT_LIBRARY_DEBUG
        NAMES libglut-debug
        PATHS ${FREEGLUT_ROOT_DIR}
        PATH_SUFFIXES Debug)

    set(FREEGLUT_LIBRARY optimized ${FREEGLUT_LIBRARY_RELEASE} debug ${FREEGLUT_LIBRARY_DEBUG})
else()
    find_library(FREEGLUT_LIBRARY glut)
endif()

find_package_handle_standard_args(FreeGLUT DEFAULT_MSG FREEGLUT_INCLUDE_DIR FREEGLUT_LIBRARY)


if(FREEGLUT_FOUND)
    set(FREEGLUT_INCLUDE_DIRS ${FREEGLUT_INCLUDE_DIR})
    set(FREEGLUT_LIBRARIES ${FREEGLUT_LIBRARY})
    message(STATUS "Found freeglut  (include: ${FREEGLUT_INCLUDE_DIR}, library: ${FREEGLUT_LIBRARY})")
    mark_as_advanced(FREEGLUT_LIBRARY_DEBUG FREEGLUT_LIBRARY_RELEASE
                     FREEGLUT_LIBRARY FREEGLUT_INCLUDE_DIR FREEGLUT_ROOT_DIR)
endif()
