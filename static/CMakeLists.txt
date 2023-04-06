# Copyright (c) 2022 Richard Kelsey. All rights reserved.

add_library (robot-sprite robot-sprite.cpp)
target_include_directories (robot-sprite PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries (robot-sprite LINK_PUBLIC robot raster util)

add_executable (voxel-data-app
  voxel-data.cpp
  voxel-data-app.cpp
)

target_link_libraries (voxel-data-app LINK_PUBLIC voxel-data-file robot json-comment raster util)

add_executable (robot-sprite-app
  robot-sprite-app.cpp
)

target_link_libraries (robot-sprite-app LINK_PUBLIC robot json-comment raster directions util robot-sprite)

add_executable (mesh-app
  mesh-app.cpp
)

target_link_libraries (mesh-app LINK_PUBLIC util mesh)

#find_package(PCL 1.3 REQUIRED)

#add_definitions(${PCL_DEFINITIONS})

#add_executable (register-app
#  register-app.cpp
#)

#target_link_libraries (register-app LINK_PUBLIC util ${PCL_DIRS})
