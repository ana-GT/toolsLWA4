cmake_minimum_required( VERSION 2.8 )
project( perceptionSystem )

# CMake module path (for ACH, Amino and SNS)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../cmake/Modules" )

# Eigen
include_directories( /usr/include/eigen3 )

# DART
find_package( DART )
include_directories( ${DART_INCLUDE_DIRS} )

# OpenCV
find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add LWA4 software paths
# amino, ach and SNS
find_package( amino REQUIRED )
include_directories( ${AMINO_INCLUDE_DIRS} )
find_package( ach REQUIRED )
include_directories( ${ACH_INCLUDE_DIRS} )
find_package( sns REQUIRED )
include_directories( ${SNS_INCLUDE_DIRS} )

# PCL
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Add levmar
add_subdirectory( tabletop_segmentation )
#add_subdirectory( getKinectTran )

if( QHULL_FOUND )
message( STATUS "QHULL WAS FOUND!!!" )
else()
message( STATUS "QHULL WAS NOT FOUND!!!")  
endif()

# Apps
add_executable( perceptionSystem perceptionSystem.cpp )
target_link_libraries( perceptionSystem tabletop_segmentation ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( perceptionSystem PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


