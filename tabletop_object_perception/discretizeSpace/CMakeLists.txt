cmake_minimum_required(VERSION 2.8)
project(PCL2Octomap)

# Include directories
include_directories( ${PROJECT_SOURCE_DIR} )
link_directories( ${CMAKE_SOURCE_DIR}/lib ) 

# PCL
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )


# OCTOMAP
find_package( octomap )
include_directories( ${OCTOMAP_INCLUDE_DIRS} )

# Library
add_library(${PROJECT_NAME} PCL2Octomap.cpp)
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} )
set_target_properties( ${PROJECT_NAME} PROPERTIES
LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Test
add_executable( storedPCD_test test/storedPCD_test.cpp )
target_link_libraries( storedPCD_test PCL2Octomap
tabletop_segmentation ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} )
set_target_properties( storedPCD_test PROPERTIES 
ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


