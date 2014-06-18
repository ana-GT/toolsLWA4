# Project
project ( crichtonSim )

set(CMAKE_INSTALL_PREFIX /usr/local )
set( CMAKE_CXX_FLAGS "-fPIC " ) 

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")

# DART
list(APPEND CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
find_package(DART REQUIRED)
include_directories(${DART_INCLUDE_DIRS})
link_directories(${DART_LIBRARY_DIRS})
 

# Find QT
find_package(Qt4 COMPONENTS QtCore QtGui Qt3Support QtXml REQUIRED)
add_definitions(${QT_DEFINITIONS})
ADD_DEFINITIONS(-DQT_PLUGIN)
ADD_DEFINITIONS(-DQT_NO_DEBUG)
ADD_DEFINITIONS(-DQT_SHARED)
include(${QT_USE_FILE})

# GRIP
find_package(GRIP REQUIRED)
include_directories(${GRIP_INCLUDE_DIRS})
set (wxWidgets_USE_LIBS base core gl)


# Add LWA4 software paths / include / libraries
# amino, ach and reflex
# Check piranhina
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



# Search all header and source files
set ( crichton_src crichtonSim.cpp )
set ( crichton_hdr crichtonSim.h )

message( STATUS "CMake of CRICHTONSIM: AMINO: ${AMINO_LIBRARIES}")

add_library( crichtonSim ${crichton_src} ${crichton_hdr} )
target_link_libraries( crichtonSim ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} ${DART_LIBRARIES} ${GRIP_LIBRARIES} )
set_target_properties( crichtonSim PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

# Add tiny test to move arm and send velocity commands
add_executable( moveArm tests/moveArm.cpp )
target_link_libraries( moveArm ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} ${DART_LIBRARIES} ${GRIP_LIBRARIES} gfortran blas lapack	)
set_target_properties( moveArm PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
