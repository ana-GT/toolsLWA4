################################################################
# piranha_control
# Evil C++ twin of pirctrl (the goodie two-shoes LISP twin)
################################################################
project( piranha_control )


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules" )

set( CMAKE_C_FLAGS "-std=gnu99" )
set( CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -fPIC" )

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

set( filter_src piranha_filter.cpp )
set( filter_hdr piranha_filter.h )

add_library( piranha_control ${control_src} ${control_hdr} )
target_link_libraries( piranha_control ${PIRANHA_LIBRARIES} ${REFLEX_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES}  ${AMINO_LIBRARIES}  blas )
set_target_properties( piranha_control PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib 
						ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

add_library( piranha_filter ${filter_src} ${filter_hdr} )
target_link_libraries( piranha_filter ${PIRANHA_LIBRARIES} ${REFLEX_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES}  ${AMINO_LIBRARIES} )
set_target_properties( piranha_filter PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib 
						ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )


add_executable( run_filter run_filter.cpp )
target_link_libraries( run_filter piranha_filter ${PIRANHA_LIBRARIES} ${REFLEX_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES}  ${AMINO_LIBRARIES}  blas lapack gfortran )
set_target_properties( run_filter PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
						ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


add_executable( run_control run_control.cpp )
target_link_libraries( run_control piranha_control ${PIRANHA_LIBRARIES} ${REFLEX_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES}  ${AMINO_LIBRARIES} blas lapack gfortran  )
set_target_properties( run_control PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin 
						ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

