/**
 * @file mindGapper.h
 * @brief Generates an object's full pointcloud from a partial view using symmetries w.r.t. a given resting plane
 */
#pragma once

#include <vector>
#include <pcl/point_cloud.h>


/**
 * @class mindGapper
 */
class mindGapper {

 public:
  mindGapper();
  ~mindGapper();
  
  /**< Set plane where the object rests and its partial cloud */
  void setPlane( std::vector<double> planeCoeff );
  bool complete( pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud ); 

 private:
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr projectToPlane( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud );


  /**< Variables */
  std::vector<double> mPlaneCoeffs;
};
