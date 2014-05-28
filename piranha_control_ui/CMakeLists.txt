project(piranha_control_ui)

#-----------------------------------------------
#                COMPILE SETTINGS
#-----------------------------------------------
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC" )
set(BUILD_FLAGS "-std=gnu++0x -Wreturn-type -Wno-enum-compare -Wall -fPIC")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")

# Define preprocessor macro for Debug printing in Debug mode
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Building in \"Debug\" mode. Defining DEBUG_BUILD macro.")
    add_definitions(-DDEBUG_BUILD)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Building in \"Release\" mode.")
else()
endif()

#-----------------------------------------------
#                FIND PACKAGES
#-----------------------------------------------
# Find QT
find_package(Qt4 COMPONENTS QtCore QtGui Qt3Support QtXml REQUIRED)
add_definitions(${QT_DEFINITIONS})
ADD_DEFINITIONS(-DQT_PLUGIN)
ADD_DEFINITIONS(-DQT_NO_DEBUG)
ADD_DEFINITIONS(-DQT_SHARED)
include(${QT_USE_FILE})

# Find OpenSceneGraph
find_package(OpenSceneGraph 2.8 REQUIRED
    COMPONENTS osg osgViewer osgManipulator osgGA osgDB)
include_directories(${OpenSceneGraph_INCLUDE_DIRS})
add_definitions(${OpenSourceGraph_DEFINITIONS})
set(OPENSCENEGRAPH_LIBRARIES ${OPENSCENEGRAPH_LIBRARIES} osgQt)

# Find Point Cloud Library
find_package(PCL 1.7 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find DART
list(APPEND CMAKE_MODULE_PATH ${CMAKE_MODLUE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
find_package(DART REQUIRED)
include_directories(${DART_INCLUDE_DIRS})
link_directories(${DART_LIBRARY_DIRS})

find_package(GRIP REQUIRED)
include_directories(${GRIP_INCLUDE_DIRS})

#-----------------------------------------------
#      INCLUDES & SUBDIRECTORY LIBRARIRES
#-----------------------------------------------
# Include top-level include directory
include_directories(include)
include_directories(${EXECUTABLE_OUTPUT_PATH})
message( STATUS "Include for ui file: ${CMAKE_BINARY_DIR}")
include_directories(${CMAKE_BINARY_DIR}/piranha_control_ui)
include_directories( ${CMAKE_SOURCE_DIR} )

# Add include directory in order to find ui_*.h files
include_directories(${CMAKE_BINARY_DIR})

# Wrap .ui files
FILE(GLOB UI_FILES ./ui/*.ui)
QT4_WRAP_UI(UI ${UI_FILES})

# Wrap .moc files
FILE(GLOB MOC_FILES ./include/*.h)
QT4_WRAP_CPP(MOC ${MOC_FILES})

# Add .qrc files
file(GLOB RC_FILES .rc/*.qrc)
QT4_ADD_RESOURCES(RESOURCES ${RC_FILES})


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



# Add source and header files
#file(GLOB srcs ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp ${CMAKE_CURRENT_LIST_DIR}/include/*.h)
file(GLOB SRCS include/*.h src/*.cpp)

# Add Qt Widgets library
add_library( piranha_control_ui SHARED ${SRCS} ${UI} ${MOC} ${RESOURCES})
target_link_libraries( piranha_control_ui piranha_control ${QT_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${DART_LIBRARIES} ${GRIP_LIBRARIES})

# Set project libraries variable
set(project_libs piranha_control_ui )
