# Copyright (c) 2022 Richard Kelsey. All rights reserved.

add_library(voxel-data-file INTERFACE)
target_include_directories(voxel-data-file INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable (core
  main.cpp
  display.cpp
  fusion.cpp
  render.cpp
  blob.cpp
  robot-projection.cpp
  registration.cpp
)

target_include_directories (core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries (core LINK_PUBLIC camera raster robot directions robot-sprite json-comment command util thread-pool GL GLU glfw GLEW rt m dl Xrandr Xext Xrender X11 pthread xcb Xau Xdmcp)
