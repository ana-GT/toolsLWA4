/**
 * @file mindGapper.cpp
 */
#include "mindGapper.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>

/**
 * @function mindGapper
 * @brief Constructor 
 */
mindGapper::mindGapper() :
  mCloud( new pcl::PointCloud<pcl::PointXYZ>() ) {

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
void mindGapper::setPlane( std::vector<double> _planeCoeffs ) {
  
  mPlaneCoeffs.resize( _planeCoeffs.size() );
  for( int i = 0; i < _planeCoeffs.size(); ++i ) {
    mPlaneCoeffs(i) = _planeCoeffs[i];
  }

  // Normalize (a,b,c), in case it has not been done already
  double norm = sqrt( pow(mPlaneCoeffs[0],2) + pow(mPlaneCoeffs[1],2) + pow(mPlaneCoeffs[2],2) );
  mPlaneCoeffs = mPlaneCoeffs / norm;
}

/**
 * @function setParams
 * @brief n: Distance steps, m: Orientation steps, dj: Distance step size, alpha: +- rotation step size
 */
void mindGapper::setParams( int _n, int _m, 
			    double _dj, double _alpha ) {
  mN = _n;
  mM = _m;
  mDj = _dj;
  mAlpha = _alpha;
}

/**
 * @function complete
 * @brief Use symmetries on plane to complete pointcloud 
 */
bool mindGapper::complete( pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud ) {

  // 0. Store
  mCloud = _cloud;

  // 1. Project pointcloud to plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  projected_cloud = projectToPlane( _cloud );

  // 2. Find eigenvalues
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud( projected_cloud );
  Eigen::Vector3f eval = pca.getEigenValues();
  Eigen::Matrix3f evec = pca.getEigenVectors();

  Eigen::Vector4d c;
  pcl::compute3DCentroid( *projected_cloud, c );
  


  std::cout << "Eigen values: \n"<< eval.transpose() << std::endl;
  std::cout << "Eigen vectors: \n"<< evec << std::endl;
  std::cout << "Centroid: \n"<< c.transpose() << std::endl;

  // 3. Pick the eigen vector most perpendicular to the viewing direction
  Eigen::Vector3d v, s;
  v << c(0), c(1), c(2);
  Eigen::Vector3d ea, eb;
  ea << (double) evec(0,0), (double) evec(1,0), (double) evec(2,0);
  eb << (double) evec(0,1), (double) evec(1,1), (double) evec(2,1);

  if( v.dot(ea) <= v.dot(eb) ) { s = ea; } 
  else { s = eb; }


  std::cout << "** \t Most perpendicular eigenvector: "<< s.transpose() << std::endl;

  // 4. Set symmetry plane coefficients
  Eigen::VectorXd sp(4);
  Eigen::Vector3d np; np = s.cross( Eigen::Vector3d(mPlaneCoeffs(0), 
						    mPlaneCoeffs(1), 
						    mPlaneCoeffs(2)) );
  np.normalize();
  sp << np(0), np(1), np(2), -1*np.dot( Eigen::Vector3d(c(0), c(1), c(2)) );

  std::cout << "\t -- Plane coefficients: "<< sp.transpose() << std::endl;

  // 5. Mirror
  pcl::PointCloud<pcl::PointXYZ>::Ptr mirrored_cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  mirrored_cloud = mirrorFromPlane( _cloud, sp, false );


  mCandidates.push_back( mirrored_cloud );

  return false;
}


/**
 * @function projectToPlane
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr mindGapper::projectToPlane( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud ) {

  // 0. Init
  pcl::PointCloud<pcl::PointXYZ>::Ptr projected( new pcl::PointCloud<pcl::PointXYZ>() );

  // 1. Project and store
  pcl::PointCloud<pcl::PointXYZ>::iterator it;
  pcl::PointXYZ p; double a;


  // Plane equation: c0*x + c1*y + c2*z + c3 = 0
  // Original 3d point P will be projected (P') into plane with normal [c0, c1, c2]
  // P' = -(c3 + c0*x + c1*y + c2*z) / (c0^2 + c1^2 + c2^2) 
  for( it = _cloud->begin(); it != _cloud->end(); ++it ) {

    p = (*it);
    a = -( mPlaneCoeffs(3) + mPlaneCoeffs(0)*p.x + mPlaneCoeffs(1)*p.y + mPlaneCoeffs(2)*p.z );

    pcl::PointXYZ pp;
    pp.x = p.x + mPlaneCoeffs(0)*a;
    pp.y = p.y + mPlaneCoeffs(1)*a;
    pp.z = p.z + mPlaneCoeffs(2)*a;

    projected->points.push_back( pp );
  }

  projected->height = projected->points.size();
  projected->width = 1;

  return projected;
}

/**
 * @function mirrorFromPlane
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr mindGapper::mirrorFromPlane( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
								 Eigen::VectorXd _plane,
								 bool _joinMirrored ) {

  // 0. Init
  pcl::PointCloud<pcl::PointXYZ>::Ptr mirrored( new pcl::PointCloud<pcl::PointXYZ>() );

  // 1. Project and store
  pcl::PointCloud<pcl::PointXYZ>::iterator it;
  pcl::PointXYZ p; double a;


  // Plane equation: c0*x + c1*y + c2*z + c3 = 0
  // Original 3d point P will be projected (P') into plane with normal [c0, c1, c2]
  // P' = -(c3 + c0*x + c1*y + c2*z) / (c0^2 + c1^2 + c2^2) 
  for( it = _cloud->begin(); it != _cloud->end(); ++it ) {

    p = (*it);
    a = -( _plane(3) + _plane(0)*p.x + _plane(1)*p.y + _plane(2)*p.z );

    pcl::PointXYZ mp;
    mp.x = p.x + 2*_plane(0)*a;
    mp.y = p.y + 2*_plane(1)*a;
    mp.z = p.z + 2*_plane(2)*a;

    mirrored->points.push_back( mp );
  }

  // If you want the output to have the whole (original + mirrored points)
  if( _joinMirrored ) {    
    for( it = _cloud->begin(); it != _cloud->end(); ++it ) {
      mirrored->points.push_back( *it );
    }
  }

  mirrored->height = mirrored->points.size();
  mirrored->width = 1;
  
  return mirrored;
}

/**
 * @function viewMirror
 */
bool mindGapper::viewMirror( int _ind ) {

  if( _ind >= mCandidates.size() ) {
    return false; 
  }

  std::cout << "-- Visualizing cloud" << std::endl;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("Mind Gap") );
  viewer->setBackgroundColor(0,0,0);
  viewer->addCoordinateSystem(1.0, 0 );
  viewer->initCameraParameters();

  // Original green, mirror blue
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color( mCloud, 0, 255, 0 );
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> mirror_color( mCandidates[_ind], 0, 0, 255 );
  viewer->addPointCloud( mCandidates[_ind], mirror_color, "mirror_cloud" );
  viewer->addPointCloud( mCloud, cloud_color, "cloud" );
  
  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
  }


  return true;

}
