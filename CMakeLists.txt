# Copyright (c) 2022 Richard Kelsey. All rights reserved.

cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project (foo)

set(CMAKE_CXX_STANDARD "17")

set(CMAKE_INSTALL_BINDIR bin)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# From https://stackoverflow.com/questions/5922016/cmake-cant-find-eigen3
find_package( PkgConfig )
#pkg_check_modules( EIGEN3 REQUIRED eigen3 )
#include_directories( ${EIGEN3_INCLUDE_DIRS} )
include_directories ( /usr/include/eigen3 )


pkg_search_module(GLFW REQUIRED glfw3)
include_directories( ${GLFW_INCLUDE_DIRS} )

# use -Og to get better debugging data, but not a lot better.
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O3 -D EIGEN_NO_DEBUG")

include_directories(
    /home/kelsey/me/micro/model2/external/json/include
)

add_subdirectory (external)
add_subdirectory (util)
add_subdirectory (robot)
add_subdirectory (camera)
add_subdirectory (static)
add_subdirectory (sim)
add_subdirectory (core)
