# Project
cmake_minimum_required( VERSION 2.8 )
project ( graspVizApp )

message (STATUS "Creating Makefile for graspVizApp")


set(CMAKE_INSTALL_PREFIX /usr/local )

# PCL
find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# DART
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
find_package(DARTExt REQUIRED)
find_package(DART REQUIRED CONFIG)

include_directories(${DARTExt_INCLUDE_DIRS})
include_directories(${DART_INCLUDE_DIRS})
link_directories(${DART_LIBRARY_DIRS})

# GRIP
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/grip")
find_package(GRIP REQUIRED)
include_directories(${GRIP_INCLUDE_DIRS})
link_directories(${GRIP_LIBRARY_DIRS})

# Search all header and source files
set ( graspViz_src graspViz.cpp graspVizApp.cpp )
set ( graspViz_hdr graspViz.h )

# Add linker directories to aoi
link_directories( /home/ana/Code/GM/lib )
include_directories( /home/ana/Code/GM )

# wxWidgets
set (wxWidgets_USE_LIBS base core gl)

find_package (wxWidgets)
if (wxWidgets_FOUND)
  include (${wxWidgets_USE_FILE})
  include_directories(${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${WXWIDGETS_SHARED_LIBRARY_DIRS})
  
  add_executable ( graspViz ${graspViz_src} ${graspViz_hdr})
  
  target_link_libraries ( graspViz grasp utils ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${wxWidgets_LIBRARIES} ${PCL_LIBRARIES} )
  set_target_properties( graspViz PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)


