
project( helpers )

link_directories( ${CMAKE_SOURCE_DIR}/lib )

# grabKinectData
add_executable( grabKinectData grabKinectData.cpp )
target_link_libraries( grabKinectData ${OpenCV_LIBS} ${PCL_LIBRARIES} )
set_target_properties( grabKinectData PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# tabletop_segmentation_test
add_executable( tabletop_segmentation_test tabletop_segmentation_test.cpp )
target_link_libraries( tabletop_segmentation_test tabletop_segmentation ${OpenCV_LIBS} ${PCL_LIBRARIES} )
set_target_properties( tabletop_segmentation_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
