#
cmake_minimum_required( VERSION 2.8 )
project( ReadMarkers )


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules" )

set( CMAKE_C_FLAGS "-std=gnu99" )
set( CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -fPIC" )

#Eigen
#find_package( PkgConfig )
#pkg_check_modules( EIGEN eigen3 )
#include_directories( ${EIGEN_INCLUDE_DIRS} )

# amino, ach and reflex
find_package( amino REQUIRED )
include_directories( ${AMINO_INCLUDE_DIRS} )
find_package( ach REQUIRED )
include_directories( ${ACH_INCLUDE_DIRS} )
find_package( sns REQUIRED )
include_directories( ${SNS_INCLUDE_DIRS} )

# Filter to read Heni's ARMarker and outputs only 3 markers important to us
add_executable( marker_filter marker_filter.cpp )
target_link_libraries( marker_filter ${SNS_LIBRARIES} ${ACH_LIBRARIES}  ${AMINO_LIBRARIES} )
set_target_properties( marker_filter PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
