### START - Adapted from argos3-examples/CMakeLists.txt

cmake_minimum_required(VERSION 2.8.12)
project(sim_epuck_loop_functions_foraging)

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(ARGOS_PREFIX "/home/eddie/argos3-dist")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
include(ARGoSCheckQTOpenGL)

# Find the GSL package
find_package(GSL REQUIRED)

# Find Lua
find_package(Lua52 REQUIRED)

# Set ARGoS include dir
include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${GSL_INCLUDE_DIR} ${LUA_INCLUDE_DIR} "/home/eddie/argos3-dist/include")

# Set ARGoS link dir
link_directories("/home/eddie/argos3-dist/lib/argos3")

### END
include_directories(${CMAKE_SOURCE_DIR}/controllers)
include_directories("../plugin/")
add_subdirectory(sim_epuck_loop_functions_foraging)
