# Copyright (c) 2022 Richard Kelsey. All rights reserved.

add_library (raster raster.cpp)
target_include_directories (raster PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(raster camera mesh fmt)

add_library (command command.cpp)
target_include_directories (command PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries (command LINK_PUBLIC gsl-lite util)

add_executable (command-test command-test.cpp)
target_include_directories(command-test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries (command-test LINK_PUBLIC command gsl-lite)

add_library (json-comment json-comment.cpp)
target_include_directories (json-comment PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries (json-comment LINK_PUBLIC)

add_library (mesh mesh.cpp)
target_include_directories (mesh PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries (mesh LINK_PUBLIC)

add_library (directions directions.cpp)
target_include_directories (directions PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries (directions LINK_PUBLIC)

add_library (util socket.cpp cruft.cpp pixel.cpp distance-map.cpp hypersphere-5-points.cpp)
target_include_directories (util PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries (util LINK_PUBLIC optionparser fmt)

add_executable (pixel-test pixel-test.cpp)
target_include_directories(pixel-test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries (pixel-test LINK_PUBLIC util)

find_package(PCL 1.2 REQUIRED)

add_executable (pcd-viewer pcd-viewer.cpp)
target_include_directories(pcd-viewer PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries (pcd-viewer LINK_PUBLIC ${PCL_LIBRARIES})
