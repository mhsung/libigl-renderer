cmake_minimum_required(VERSION 3.1)

option(OFFSCREEN        "Use OffscreenRenderer" ON)
option(USE_OSMESA       "Use OSMesa"            OFF)

if (OFFSCREEN)
    project(OffscreenRenderer)
    message(STATUS "== Use Offscreen Renderer ==")
else()
    project(LibiglViewer)
    message(STATUS "== Use Libigl Mesh Renderer ==")
endif()

set(CMAKE_BUILD_TYPE    "RelWithDebInfo")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# libigl options: choose your dependencies (by default everything is OFF except opengl)
option(LIBIGL_WITH_OPENGL        "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW   "Use GLFW"           ON)
option(LIBIGL_WITH_EMBREE        "Use Embree"         ON)
option(LIBIGL_WITH_PNG           "Use PNG"            ON)

find_package(LIBIGL REQUIRED)

include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/base"
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

FILE(GLOB SRC_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/base/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/*/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/*/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/*/*.cpp")

if (USE_OSMESA)
    add_definitions(-DGLFW_USE_OSMESA)
    find_package(OSMesa REQUIRED)
    if (NOT OSMESA_FOUND)
        message(FATAL_ERROR "OSMesa not found")
    endif()
endif()

if (OFFSCREEN)
    add_definitions(-DOFFSCREEN)
    include_directories(
        ${OSMESA_INCLUDE_DIR}
        "${CMAKE_CURRENT_SOURCE_DIR}/renderers/GL"
        "${CMAKE_CURRENT_SOURCE_DIR}/renderers/offscreen")
    FILE(GLOB RENDERER_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/renderers/GL/*.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/renderers/offscreen/*.cc")
else()
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/renderers/libigl")
    FILE(GLOB RENDERER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/renderers/libigl/*.cc")
endif()

# GFlags
find_package(GFlags REQUIRED)
if (NOT GFLAGS_FOUND)
    message(FATAL_ERROR "gflags not found")
endif()
include_directories(${GFLAGS_INCLUDE_DIRS})

# Glog
find_package(Glog REQUIRED)
if (NOT GLOG_FOUND)
    message(FATAL_ERROR "glog not found")
endif()
include_directories(${GLOG_INCLUDE_DIRS})

# HDF5
find_package(HDF5 REQUIRED CXX)
if (NOT HDF5_FOUND)
    message(FATAL_ERROR "hdf5 not found")
endif()
include_directories(${HDF5_INCLUDE_DIRS})
link_directories(${HDF5_LIBRARY_DIRS})

# Add project files
add_executable(${PROJECT_NAME} ${SRC_FILES} ${RENDERER_FILES})
target_link_libraries(${PROJECT_NAME} igl::core igl::embree igl::png)

if (GLFW_USE_OSMESA)
    target_link_libraries(${PROJECT_NAME} ${OSMESA_LIBRARY})
else ()
    target_link_libraries(${PROJECT_NAME} igl::opengl igl::opengl_glfw)
endif()

target_link_libraries(${PROJECT_NAME}
    ${GFLAGS_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${HDF5_CXX_LIBRARIES}
)
