# Copyright (c) 2022 Richard Kelsey. All rights reserved.

add_library (camera
  camera.cpp)

target_include_directories (camera PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries (camera LINK_PUBLIC crc32 util)
