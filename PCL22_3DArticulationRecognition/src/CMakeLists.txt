cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(articulation_detection)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (articulation_detection articulation_detection.cpp)
target_link_libraries (articulation_detection ${PCL_LIBRARIES})