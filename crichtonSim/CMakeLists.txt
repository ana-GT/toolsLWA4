# Project
project ( crichtonSim )

set(CMAKE_INSTALL_PREFIX /usr/local )
set( CMAKE_CXX_FLAGS "-fPIC " ) 

# DART
list(APPEND CMAKE_MODULE_PATH ${CMAKE_MODLUE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
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



# Search all header and source files
set ( crichton_src crichtonSim.cpp )
set ( crichton_hdr crichtonSim.h )

add_library( crichtonSim ${crichton_src} ${crichton_hdr} )
target_link_libraries( crichtonSim ${DART_LIBRARIES} ${GRIP_LIBRARIES} )
set_target_properties( crichtonSim PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
