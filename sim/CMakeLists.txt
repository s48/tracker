# Copyright (c) 2022 Richard Kelsey. All rights reserved.

add_executable (sim
  main.cpp
)

target_include_directories (sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries (sim camera optionparser json-comment raster robot command util thread-pool pthread tinyxml2 rt)
