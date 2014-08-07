
project( helpers )

link_directories( ${CMAKE_SOURCE_DIR}/lib )

# grabKinectData
add_executable( grabKinectData grabKinectData.cpp )
target_link_libraries( grabKinectData ${OpenCV_LIBS} ${PCL_LIBRARIES} )
set_target_properties( grabKinectData PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# grabPCD
add_executable( grabPCD grabPCD.cpp )
target_link_libraries( grabPCD ${OpenCV_LIBS} ${PCL_LIBRARIES} )
set_target_properties( grabPCD PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# showPCD
add_executable( showPCD showPCD.cpp )
target_link_libraries( showPCD ${OpenCV_LIBS} ${PCL_LIBRARIES} )
set_target_properties( showPCD PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# concave_hull_2d_test
add_executable( concave_hull_2d concave_hull_2d.cpp )
target_link_libraries( concave_hull_2d ${OpenCV_LIBS} ${PCL_LIBRARIES} )
set_target_properties( concave_hull_2d PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


# tabletop_segmentation_test
add_executable( tabletop_segmentation_test tabletop_segmentation_test.cpp )
target_link_libraries( tabletop_segmentation_test tabletop_segmentation ${OpenCV_LIBS} ${PCL_LIBRARIES} )
set_target_properties( tabletop_segmentation_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# tabletop_symmeetry_test
add_executable( tabletop_symmetry_test tabletop_symmetry_test.cpp )
target_link_libraries( tabletop_symmetry_test mindGapper ${OpenCV_LIBS} ${PCL_LIBRARIES} )
set_target_properties( tabletop_symmetry_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# test_image
add_executable( testImage testImage.cpp )
target_link_libraries( testImage  ${OpenCV_LIBS} ${PCL_LIBRARIES} )
set_target_properties( testImage PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# dt_test
add_executable( dt_test dt_test.cpp )
target_link_libraries( dt_test  dt ${OpenCV_LIBS} )
set_target_properties( dt_test PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
