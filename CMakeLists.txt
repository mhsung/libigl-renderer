cmake_minimum_required(VERSION 3.1)

# Set whether use OSMesa or GLUT
option(USE_OSMESA   "Use OSMesa Mesh Renderer"  ON)
option(USE_GLUT     "Use GLUT Mesh Renderer"    OFF)

if (USE_OSMESA)
  project(OSMesaRenderer)
  message(STATUS "== Use OSMesa Mesh Renderer ==")
elseif (USE_GLUT)
  project(GlutViewer)
  message(STATUS "== Use GLUT Mesh Renderer ==")
else()
  project(LibiglViewer)
  message(STATUS "== Use Libigl Mesh Renderer ==")
endif()

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

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
elseif (USE_GLUT)
  add_definitions(-DUSE_GLUT)
  find_package(OpenGL REQUIRED)
  if (NOT OPENGL_FOUND)
    message(FATAL_ERROR "OpenGL not found")
  endif()
  find_package(FreeGLUT REQUIRED)
  if (NOT FREEGLUT_FOUND)
    message(FATAL_ERROR "FreeGLUT not found")
  endif()
  include_directories(
    ${OPENGL_INCLUDE_DIR}
    ${FREEGLUT_INCLUDE_DIR}
    "${CMAKE_CURRENT_SOURCE_DIR}/renderers/GL"
    "${CMAKE_CURRENT_SOURCE_DIR}/renderers/glut")
  FILE(GLOB RENDERER_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/renderers/GL/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/renderers/glut/*.cc")
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

if (USE_OSMESA)
  target_link_libraries(${PROJECT_NAME} ${OSMESA_LIBRARY})
elseif (USE_GLUT)
  target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${FREEGLUT_LIBRARIES})
else ()
  target_link_libraries(${PROJECT_NAME} igl::opengl igl::opengl_glfw)
endif()

target_link_libraries(${PROJECT_NAME}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${HDF5_CXX_LIBRARIES}
)
