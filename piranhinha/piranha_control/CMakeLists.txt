################################################################
# piranha_control
# Evil C++ twin of pirctrl (the goodie two-shoes LISP twin)
################################################################
project( piranha_control )


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
message( STATUS "PIRANHA INCLUDE DIRS: ${PIRANHA_INCLUDE_DIRS}" )

include_directories( ${CMAKE_SOURCE_DIR}/include )

set( control_src piranha_control.cpp )
set( control_hdr piranha_control.h )

add_library( ${PROJECT_NAME} ${control_src} ${control_hdr} )
target_link_libraries( ${PROJECT_NAME} ${PIRANHA_LIBRARIES} ${REFLEX_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES}  ${AMINO_LIBRARIES}   )
set_target_properties( ${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib 
						ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
