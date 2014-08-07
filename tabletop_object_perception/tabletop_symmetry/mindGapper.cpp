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
  mCloud( new pcl::PointCloud<pcl::PointXYZ>() ),
  mProjected( new pcl::PointCloud<pcl::PointXYZ>() ){

  // Hard-coded
  mWidth = 640;
  mHeight = 480;
  mF = 525;
  mCx = 320; mCy = 240;
  

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

  // 0. Store cloud and its mask
  mCloud = _cloud;
  this->generate2DMask( mCloud,
			mMarkMask,
			mDepthMask );


  // 1. Project pointcloud to plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  projected_cloud = projectToPlane( _cloud );
  mProjected = projected_cloud;


  // 2. Find eigenvalues
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud( projected_cloud );
  Eigen::Vector3f eval = pca.getEigenValues();
  Eigen::Matrix3f evec = pca.getEigenVectors();

  Eigen::Vector4d c;
  pcl::compute3DCentroid( *projected_cloud, c );
  
  mC << c(0), c(1), c(2);
  mEa << (double) evec(0,0), (double) evec(1,0), (double) evec(2,0);
  mEb << (double) evec(0,1), (double) evec(1,1), (double) evec(2,1);


  // 3. Pick the eigen vector most perpendicular to the viewing direction
  Eigen::Vector3d v, s, s_sample;
  v = mC;

  if( abs(v.dot(mEa)) <= abs(v.dot(mEb)) ) { s = mEa; } 
  else { s = mEb; }

  
  // 4. Get rotation samples
  Eigen::Vector3d Np; 
  Np << mPlaneCoeffs(0), mPlaneCoeffs(1), mPlaneCoeffs(2); 
  double ang, dang;

  Eigen::VectorXd sp(4);
  Eigen::Vector3d np, cp, dir;

  dang = 2*mAlpha / (double) mM;
    
  for( int i = 0; i < mM; ++i ) {
        
    ang = -mAlpha +i*dang;
    s_sample = Eigen::AngleAxisd( ang, Np )*s;
    np = s_sample.cross( Np );
    np.normalize();
    
    for( int j = 0; j < mN; ++j ) {

      if( np.dot(v) > -np.dot(v) ) { dir = np; } else { dir = -np; }
      cp = mC + dir*mDj*j;

      //Set symmetry plane coefficients
      sp << np(0), np(1), np(2), -1*np.dot( cp );

      // 5. Mirror
      pcl::PointCloud<pcl::PointXYZ>::Ptr mirrored_cloud( new pcl::PointCloud<pcl::PointXYZ>() );
      mirrored_cloud = mirrorFromPlane( _cloud, sp, false );
      mCandidates.push_back( mirrored_cloud );
    } // end for N    
  } // end for M


  // 6. Evaluate
  for( int i = 0; i < mCandidates.size(); ++i ) {

    cv::Mat mark_i = cv::Mat::zeros( mHeight, mWidth, CV_8UC3 );


    // Check inliers and outliers
    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    int px, py; pcl::PointXYZ P;
    int outside = 0; int front = 0; int behind = 0;
    for( it = mCandidates[i]->begin(); 
	 it != mCandidates[i]->end(); ++it ) {
      P = (*it);
      px = round( mF*(P.x / P.z) + mCx );
      py = round( mF*(P.y / P.z) + mCy );
      

      if( px < 0 || px >= mWidth ) { return false; }
      if( py < 0 || py >= mHeight ) { return false; }
 
      // Outside segmented mask - RED
      if( mMarkMask.at<uchar>(py,px) != 255 ) {
	cv::Vec3b col(0,0,255);
	mark_i.at<cv::Vec3b>(py,px) = col;
	outside++;
      } 
      // If inside
      else {
	// If in front of visible BLUE
	if( (float)P.z < mDepthMask.at<float>(py,px) ) {
	  cv::Vec3b col(255,0,0);
	  mark_i.at<cv::Vec3b>(py,px) = col;
	  front++;
	} else {
	  // If behind - GREEN
	  cv::Vec3b col(0,255,0);
	  mark_i.at<cv::Vec3b>(py,px) = col;
	  behind++;
	}
      }
    

    } // for it
    
    char name[50];
    std::cout << " Candidate ["<<i<<"]: Outside: "<< outside<<" front: "<< front << " and behind: "<< behind << " - TOTAL: "<< outside + front + behind <<std::endl;
    sprintf( name, "candidate_%d.png", i );
    imwrite( name, mark_i );
    
  } // for each candidate

  // DEBUG

  int out; int in;
  for( int i = 0; i < mCandidates.size(); ++i ) {
    out = 0; in = 0;
    cv::Mat debug_i = cv::Mat::zeros( mHeight, mWidth, CV_8UC3 );

    cv::Mat candMark, candDepth;
    generate2DMask( mCandidates[i],
		    candMark, candDepth );
    

    cv::Vec3b ORIGINAL( 0,255, 0 );
    cv::Vec3b MIRROR( 0, 0, 255 );
    for( int j = 0; j < mHeight; ++j ) {
      for( int k = 0; k < mWidth; ++k ) {
	if( mMarkMask.at<uchar>(j,k) == 255 ) {
	  debug_i.at<cv::Vec3b>(j,k) = ORIGINAL;
	}
      }
    }

    for( int j = 0; j < mHeight; ++j ) {
      for( int k = 0; k < mWidth; ++k ) {
	if( candMark.at<uchar>(j,k) == 255 ) {
	  if( mMarkMask.at<uchar>(j,k) == 255 ) { in++; continue; }
	  else { debug_i.at<cv::Vec3b>(j,k) = MIRROR; out++; }

	}
      }
    }
    std::cout<<"DEBUG ["<<i<<"]: IN: "<<in<<" OUT: "<< out <<" TOTAL: "<< in + out << std::endl;
    char debugName[50];
    sprintf( debugName, "debug_%d.png", i );
    imwrite( debugName, debug_i );


    
  } // for each candidate



  return true;
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

/**
 * @function viewInitialParameters
 */
bool mindGapper::viewInitialParameters() {


  std::cout << "-- Visualizing initial parameters" << std::endl;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("Initial") );
  viewer->setBackgroundColor(0,0,0);
  viewer->addCoordinateSystem(1.0, 0 );
  viewer->initCameraParameters();

  // Original green, mirror blue
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color( mCloud, 0, 255, 0 );
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> projected_color( mProjected, 0, 0, 255 );
  viewer->addPointCloud( mProjected, projected_color, "projected" );
  viewer->addPointCloud( mCloud, cloud_color, "cloud" );
  
  // Center red ball
  pcl::PointXYZ c;
  c.x = mC(0); c.y = mC(1); c.z = mC(2);
  double r, g, b;
  r = 1; g = 0; b = 0;
  viewer->addSphere( c, 0.015, r, g, b, "centroid" );

  // Draw Eigen vectors: ea magenta, eb yellow
  pcl::PointXYZ pea, peb;
  double l = 0.20;
  pea.x = c.x + mEa(0)*l;   pea.y = c.y + mEa(1)*l;   pea.z = c.z + mEa(2)*l;
  peb.x = c.x + mEb(0)*l;   peb.y = c.y + mEb(1)*l;   peb.z = c.z + mEb(2)*l;

  r = 1.0; g = 0.0; b = 1.0;
  viewer->addLine( c, pea, r, g, b, "ea", 0 );
  r = 1.0; g = 1.0; b = 0.0;
  viewer->addLine( c, peb,  r, g, b, "eb", 0 );

  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
  }

  
  return true;

}




/**
 * @function view2DMask
 */
bool mindGapper::generate2DMask( pcl::PointCloud<pcl::PointXYZ>::Ptr _segmented_cloud,
				 cv::Mat &_markMask,
				 cv::Mat &_depthMask ) {

  _markMask = cv::Mat::zeros( mHeight, mWidth, CV_8UC1 );
  _depthMask = cv::Mat::zeros( mHeight, mWidth, CV_32FC1 );
  
  
  // Color the segmented crap
  pcl::PointCloud<pcl::PointXYZ>::iterator it;
  pcl::PointXYZ P;
  int px; int py;
  
  int repeated = 0;

  for( it = _segmented_cloud->begin(); 
       it != _segmented_cloud->end(); ++it ) {
    P = (*it);
    px = round( mF*(P.x / P.z) + mCx );
    py = round( mF*(P.y / P.z) + mCy );

    if( px == -1 ) { px = 0; }
    if( py == -1 ) { py = 0; }
    if( px == mWidth ) { px = mWidth - 1; }
    if( py == mHeight ) { py = mHeight - 1; }
    

    if( px < 0 || px >= mWidth ) { return false; }
    if( py < 0 || py >= mHeight ) { return false; }
    

    if(  _markMask.at<uchar>(py,px) == 255 ) {
      repeated++;
    }
    _markMask.at<uchar>(py,px) = 255;
    _depthMask.at<float>(py,px) = (float)P.z;
  }
  
  std::cout << "Repeated points: "<< repeated << std::endl;

  return true;
}



