#*******************
# control
#*******************
cmake_minimum_required( VERSION 2.8 )
project( control )
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules" )
set( CMAKE_C_FLAGS "-std=gnu99" )

# amino, ach and reflex
find_package( amino REQUIRED )
include_directories( ${AMINO_INCLUDE_DIRS} )
find_package( ach REQUIRED )
include_directories( ${ACH_INCLUDE_DIRS} )
find_package( sns REQUIRED )
include_directories( ${SNS_INCLUDE_DIRS} )
find_package( piranha REQUIRED )

# Add tests
add_subdirectory( tests )