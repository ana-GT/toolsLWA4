# *********************************************************
# CMake for piraninha 
# piranhinha is the same as piranha + extra goodies to
# make it more user-friendly for grip2
# *********************************************************
cmake_minimum_required( VERSION 2.8 )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules" )
set( CMAKE_C_FLAGS "-std=gnu99" )

# amino, ach and reflex
find_package( amino REQUIRED )
include_directories( ${AMINO_INCLUDE_DIRS} )
find_package( ach REQUIRED )
include_directories( ${ACH_INCLUDE_DIRS} )
find_package( reflex REQUIRED )
include_directories( ${REFLEX_INCLUDE_DIRS} )
find_package( sns REQUIRED )
include_directories( ${SNS_INCLUDE_DIRS} )
find_package( piranha REQUIRED )
include_directories( ${PIRANHA_INCLUDE_DIRS} )

include_directories( ${CMAKE_SOURCE_DIR}/include )

# Add piranha_control
add_subdirectory( piranha_control )

