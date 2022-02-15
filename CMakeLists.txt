cmake_minimum_required(VERSION 3.16)

# Set whether use OSMesa.
option(USE_OSMESA   "Use OSMesa Mesh Renderer"  ON)

if (USE_OSMESA)
  project(OSMesaRenderer)
  message(STATUS "== Use OSMesa Mesh Renderer ==")
else()
  project(LibiglViewer)
  message(STATUS "== Use Libigl Mesh Renderer ==")
endif()

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(LIBIGL_EMBREE  "igl::embree" ON)
option(LIBIGL_GLFW    "igl::glfw"   ON)
option(LIBIGL_OPENGL  "igl::opengl" ON)
option(LIBIGL_PNG     "igl::png"    ON)
include(libigl)

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
  add_definitions(-DUSE_OSMESA)
  find_package(OSMesa REQUIRED)
  if (NOT OSMESA_FOUND)
    message(FATAL_ERROR "OSMesa not found")
  endif()
  include_directories(
    ${OSMESA_INCLUDE_DIR}
    "${CMAKE_CURRENT_SOURCE_DIR}/renderers/GL"
    "${CMAKE_CURRENT_SOURCE_DIR}/renderers/osmesa")
  FILE(GLOB RENDERER_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/renderers/GL/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/renderers/osmesa/*.cc")
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
target_link_libraries(${PROJECT_NAME} igl::embree igl::png)

if (USE_OSMESA)
  target_link_libraries(${PROJECT_NAME} ${OSMESA_LIBRARY})
else ()
  target_link_libraries(${PROJECT_NAME} igl::opengl igl::glfw)
endif()

target_link_libraries(${PROJECT_NAME}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${HDF5_CXX_LIBRARIES}
)
