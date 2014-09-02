cmake_minimum_required( VERSION 2.8 )
project( getKinectTran )

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

# Add project
add_executable( ${PROJECT_NAME} getKinectTran.cpp )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( ${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( moveEE_predef moveEE_predef.cpp )
target_link_libraries( moveEE_predef ${DART_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( moveEE_predef PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( testCalib testCalib.cpp )
target_link_libraries( testCalib ${DART_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} lapack blas gfortran )
set_target_properties( testCalib PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (testICP testICP.cpp)
target_link_libraries (testICP ${PCL_LIBRARIES})
