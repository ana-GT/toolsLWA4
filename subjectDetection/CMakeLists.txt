# CMake
cmake_minimum_required( VERSION 2.8 )

# Add paths for OpenNI cmake file
set( CMAKE_MODULE_PATH  ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules" )

# Find Eigen
find_package( PkgConfig )
pkg_check_modules( EIGEN eigen3 )
include_directories( ${EIGEN_INCLUDE_DIRS} )

# OpenNI
find_package( OpenNI )	
include_directories( ${OPENNI_INCLUDE_DIRS} )	

# Ach
find_package( ach )
include_directories( ${ACH_INCLUDE_DIRS} )

# Include Locals
include_directories( ${CMAKE_SOURCE_DIR}/include )

#
find_package( GLUT )
find_package( OpenGL )
include_directories( ${GLUT_INCLUDE_DIRS} )
include_directories( ${OPENGL_INCLUDE_DIRS} )

# Add library
add_library( subjectTracking src/subjectTracking.cpp )
target_link_libraries( subjectTracking ${OPENNI_LIBRARIES} ${ACH_LIBRARIES} )
set_target_properties( subjectTracking PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/library )


# Add samples
add_executable( NiSimpleSkeleton tests/NiSimpleSkeleton.cpp )
target_link_libraries( NiSimpleSkeleton ${OPENNI_LIBRARIES} )
set_target_properties( NiSimpleSkeleton PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

include_directories( ${CMAKE_SOURCE_DIR}/tests/NiUserTracker )

add_executable( NiUserTracker tests/NiUserTracker/main.cpp
tests/NiUserTracker/opengles.cpp tests/NiUserTracker/SceneDrawer.cpp )
target_link_libraries( NiUserTracker ${OPENNI_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} )
set_target_properties( NiUserTracker PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )



add_executable( subjectTracking_main tests/subjectTracking_main.cpp )
target_link_libraries( subjectTracking_main subjectTracking ${OPENNI_LIBRARIES} )
set_target_properties( subjectTracking_main PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


