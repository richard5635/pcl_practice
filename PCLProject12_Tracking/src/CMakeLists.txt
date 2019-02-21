cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

project(tracking_sample)
add_executable (tracking_sample kinect2_grabber.h tracking_sample.cpp)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "tracking_sample" )

find_package(PCL 1.7 REQUIRED)
find_package( KinectSDK2 REQUIRED )

if( PCL_FOUND AND KinectSDK2_FOUND )
 # Additional Include Directories
 include_directories(${PCL_INCLUDE_DIRS})
 include_directories( ${KinectSDK2_INCLUDE_DIRS} )

 # Additional Library Directories
 link_directories(${PCL_LIBRARY_DIRS})
 link_directories( ${KinectSDK2_LIBRARY_DIRS} )

 # Preprocessor Definitions
 add_definitions(${PCL_DEFINITIONS})

 # Additional Dependencies
 target_link_libraries (tracking_sample ${KinectSDK2_LIBRARIES} )
 target_link_libraries (tracking_sample ${PCL_LIBRARIES})
endif()

