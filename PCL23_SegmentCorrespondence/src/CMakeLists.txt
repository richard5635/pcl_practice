cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(segment_correspondence)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (segment_correspondence segment_correspondence.cpp)
target_link_libraries (segment_correspondence ${PCL_LIBRARIES})