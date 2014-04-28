# CMake for helpers
cmake_minimum_required( VERSION 2.8 )

project( tabletop_object_perception )

# Source files
include_directories( ${CMAKE_SOURCE_DIR} )

# PCL
find_package( PCL 1.7 REQUIRED )

include_directories(  ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


# Add subdirectories
add_subdirectory( helpers )
add_subdirectory( tabletop_object_detector )
