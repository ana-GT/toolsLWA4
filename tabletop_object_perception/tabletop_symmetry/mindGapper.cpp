/**
 * @file mindGapper.cpp
 */
#include "mindGapper.h"


/**
 * @function mindGapper
 * @brief Constructor 
 */
mindGapper::mindGapper() {

}

/**
 * @function ~mindGapper
 * @brief Destructor 
 */
mindGapper::~mindGapper() {

}

/**
 * @function setPlane
 * @brief Set resting plane and object to complete based on symmetry
 */
mindGapper::setPlane( std::vector<double> _planeCoeffs ) {
  
  for( int i = 0; i < _planeCoeffs.size(); ++i ) {
    mPlaneCoeffs[i] = _planeCoeffs[i];
  }
}

/**
 * @function complete
 * @brief Use symmetries on plane to complete pointcloud 
 */
bool mindGapper::complete( pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud ) {

  return false;
}


/**
 * @function projectToPlane
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr mindGapper::projectToPlane( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud );

