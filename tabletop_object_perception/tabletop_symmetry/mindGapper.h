/**
 * @file mindGapper.h
 * @brief Generates an object's full pointcloud from a partial view using symmetries w.r.t. a given resting plane
 */
#pragma once

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//-- OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @class mindGapper
 */
class mindGapper {

 public:
  mindGapper();
  ~mindGapper();
  
  /**< Set plane where the object rests and its partial cloud */
  void setPlane( std::vector<double> planeCoeff );
  void setParams( int _n = 6, int _m = 5, 
		  double _dj = 0.01, double _alpha = 20.0*M_PI / 180.0 );
  bool complete( pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud ); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr getCandidate(int _ind) { return mCandidates[_ind]; }
  bool generate2DMask(  pcl::PointCloud<pcl::PointXYZ>::Ptr _segmented_cloud,
			cv::Mat &_markMask,
			cv::Mat &_depthMask );
  cv::Mat get2DMask() { return mMarkMask; }
  


  // DEBUG FUNCTIONS
  bool viewMirror( int _ind );
  bool viewInitialParameters();

 private:
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr projectToPlane( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud );
  pcl::PointCloud<pcl::PointXYZ>::Ptr mirrorFromPlane( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
						       Eigen::VectorXd _plane,
						       bool _joinMirrored = true );

  pcl::PointCloud<pcl::PointXYZ>::Ptr mProjected;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> mCandidates;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud;

  /**< Variables */
  Eigen::VectorXd mPlaneCoeffs;
  int mN;
  int mM;
  double mDj;
  double mAlpha;
  
  // Mask variables
  double mF, mCx, mCy;
  cv::Mat mMarkMask; cv::Mat mDepthMask;
  int mWidth; int mHeight;

  Eigen::Vector3d mC;
  Eigen::Vector3d mEa, mEb;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
};
