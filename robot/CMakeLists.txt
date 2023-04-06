# Copyright (c) 2022 Richard Kelsey. All rights reserved.

add_library (robot
  urdf.cpp
  robot.cpp
)

target_include_directories(robot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(robot fmt tinyxml2 mesh)

add_executable (urdf-test
  test.cpp
)

target_include_directories(urdf-test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(urdf-test fmt robot util)
