/**
 * @file getKinectTran
 * @brief Get transformation from Kinect w.r.t. the robot world origin
 * @brief using a bunch of correspondences and SVD for minimization
 */

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <fstream>

#include <ach.h>
#include <sns.h>

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pk;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pw;

cv::Mat rgbImg;
cv::Mat pclMap;

ach_channel_t ee_pos_chan;

/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int flags, void* userdata );
void startComm( int state, void* userdata );
void process( int state, void* userdata );
void icpApproach();
void svdApproach();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI );
//  capture.set( cv::CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, cv::CAP_OPENNI_SXGA_15HZ );
  //capture.open( cv::CAP_OPENNI );
  
  if( !capture.isOpened() ) {
    std::cout << "/t * Could not open the capture object"<<std::endl;
    return -1;
  }

  // Set control panel
  cv::namedWindow( "BGR", cv::WINDOW_AUTOSIZE );
  //  cv::CreateTrackbar("track1", "BGR", &value, 255, NULL );
  
  cv::createButton( "Start comm", startComm, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  cv::createButton( "Process", process, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  

  // Set mouse callback 
  cv::setMouseCallback( "BGR", onMouse, 0 );


  // Loop
  for(;;) {

    if( !capture.grab() ) {
      std::cout << "\t * [DAMN] Could not grab a frame" << std::endl;
      return -1;
    }

    capture.retrieve( rgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    cv::imshow( "BGR", rgbImg );
    
      
    capture.retrieve( pclMap, cv::CAP_OPENNI_POINT_CLOUD_MAP );
    

    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * [PRESSED ESC] Finishing the program \n");
      break;
    }

  } // end for
  
  return 0;
}

/**
 * @function onMouse
 * @brief Stores the current position of clicked point and calculates the world position from the robot's kinematics
 */
static void onMouse( int event, int x, int y, int, void* ) {
  
  if( event != cv::EVENT_LBUTTONDOWN ) {
    return;
  }

  // Get (X,Y,Z) from Kinect
  cv::Point3f p;
  Eigen::Vector3d pk;
  
  p = pclMap.at<cv::Point3f>(y,x);
  pk(0) = (double)p.x; pk(1) = (double)p.y; pk(2) = (double)p.z;

	// DEBUG
	printf("Clicked in (%d,%d), Kinect coordinates: %f %f %f \n", x, y, pk(0), pk(1), pk(2));

	// CHECK WORST ERROR CASE
	p = pclMap.at<cv::Point3f>(x,y);
  pk(0) = (double)p.x; pk(1) = (double)p.y; pk(2) = (double)p.z;

	// DEBUG 2
	printf("[Second interpretation] Clicked in (%d,%d), Kinect coordinates: %f %f %f \n", x, y, pk(0), pk(1), pk(2));

  // Get (X,Y,Z) from robot kinematics
  Eigen::Vector3d pw;
  double msg[3];
  size_t frame_size;
  ach_status r;
  
  r = ach_get( &ee_pos_chan,
	       msg,
	       sizeof(msg),
	       &frame_size, NULL, ACH_O_LAST );

  if( r != ACH_MISSED_FRAME && r != ACH_OK ) {
    printf("\t * [BAD] Did not receive updated EE pos - NO STORING POINT \n");
    return;
  }
  
  for( int i = 0; i < 3; ++i ) { pw(i) = msg[i]; }
  printf( "Pixel: %d, %d; \n", x, y );
  printf( "Pk[%d] << %f, %f, %f; Pr[%d] << %f, %f, %f;\n", Pk.size(), 
	  pk(0), pk(1), pk(2), Pw.size(), pw(0), pw(1), pw(2) );

  // Store it
  Pk.push_back(pk);
  Pw.push_back(pw);
  printf("*\t * Stored %d points so far \n", Pk.size() );
  
}

/**
 * @function startComm
 */
void startComm( int state, void* userdata ) {

  printf("\t * Start communication \n");
  sns_init();
  sns_start();

  sns_chan_open( &ee_pos_chan, "ee-pos", NULL );  
  printf("\t * [OK] Communication stablished and ready to go \n");

}

/**
 * @function process
 */
void process( int state, void* userdata ) {

  std::ofstream file("data.txt");

  if(file.is_open())
  {
    for(int i =0; i < Pk.size(); i++)
    {
	file << Pk[i](0) << " " << Pk[i](1) << " " << Pk[i](2) << " ";
	file << Pw[i](0) << " " << Pw[i](1) << " " << Pw[i](2) << " " << std::endl;
    }
	file.close();
  }


  if( Pk.size() < 4) {
    printf("\t * [ERROR] You need at least 4 points but I DO RECOMMEND YOU TO GET AS MANY AS POSSIBLE (AT LEAST 12)!\n");
    return;
  }

  icpApproach();
  svdApproach();
}


/**
 * @function icpApproach
 * @brief Make sure you use a lot of points (with 7 points I got an error as high as 44 cm. With 12 points, the max. error was 2.5cm
 */
void icpApproach() {

  std::cout << "ICP"<< std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the CloudIn data
  cloud_in->width    = Pk.size();
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);

  cloud_out->width    = Pw.size();
  cloud_out->height   = 1;
  cloud_out->is_dense = false;
  cloud_out->points.resize (cloud_out->width * cloud_out->height);


  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = Pk[i](0);
    cloud_in->points[i].y = Pk[i](1);
    cloud_in->points[i].z = Pk[i](2);

    cloud_out->points[i].x = Pw[i](0);
    cloud_out->points[i].y = Pw[i](1);
    cloud_out->points[i].z = Pw[i](2);

  }

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  Eigen::Matrix4f tf = icp.getFinalTransformation();
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  std::cout << "\t * ICP converged? :" << icp.hasConverged() << ". Score: " <<
    icp.getFitnessScore() << std::endl;
  std::cout << "\t * Transformation for Kinect in world frame: \n"<< tf << std::endl;


  Eigen::Matrix4d Tf;
  for( int i = 0; i < 4; i++ ) {
    for( int j = 0; j < 4; j++ ) {
      Tf(i,j) = (double) tf(i,j);
    }
  }
 
  std::cout <<"Check: "<<std::endl;
  for( int i = 0; i < Pk.size(); ++i ) {
    Eigen::Vector3d pt;
    pt = ( (Tf.block(0,0,3,3))*Pk[i] + Tf.block(0,3,3,1) );
 
    std::cout << " Pw["<<i<<"]: "<< Pw[i].transpose() <<
      " , Tf(Pk[i]): "<< pt.transpose() <<". Error: "<< (Pw[i] - pt).norm() << std::endl;
  }
  
}


/**
 * @function svdApproach
 */
void svdApproach() {

  std::cout << "SVD Result"<< std::endl;

  int numMatches = Pk.size();

  // 1. Compute the weighted centroids of both kinect and world sets
  // (all points same weight in our case)
  Eigen::Vector3d xm; xm << 0, 0, 0; // Points from Kinect
  Eigen::Vector3d ym; ym << 0, 0, 0; // Points from kinematics

  for( int i = 0; i < numMatches; ++i ) {    
    xm += Pk[i];
    ym += Pw[i];
  }
  xm = xm / numMatches; ym = ym / numMatches;

  // 2. Compute centered vectors
  Eigen::MatrixXd X(3,numMatches);
  Eigen::MatrixXd Y(3,numMatches);	

  for( int i = 0; i < numMatches; ++i ) {
    X.col(i) = Pk[i] - xm;
    Y.col(i) = Pw[i] - ym;
  }

  // 3. Compute the 3x3 covariance matrix
  Eigen::Matrix3d S;
  S = X*(Y.transpose()); // X*W*Yt -> W is identity


  // 4. Compute the singular value decomposition
  Eigen::JacobiSVD<Eigen::MatrixXd> svd( S, Eigen::ComputeThinU | Eigen::ComputeThinV );
  Eigen::Matrix3d U; Eigen::Matrix3d V;
  U = svd.matrixU(); V = svd.matrixV();
  
  Eigen::Matrix3d temp; temp = V*(U.transpose()); 
  Eigen::Matrix3d M; M.setIdentity(); M(2,2) = temp.determinant();
  std::cout << "This should be 1: "<< M(2,2) << std::endl;
  Eigen::Matrix3d Rot;
  Rot = V*M*(U.transpose());

  Eigen::Vector3d trans;
  trans = ym - Rot*xm;

  Eigen::Matrix4d Tf;
  Tf.block(0,0,3,3) = Rot;
  Tf.block(0,3,3,1) = trans;

  std::cout << "SVD Resulting Transformation: \n"<< Tf << std::endl;

  std::cout <<"Check: "<<std::endl;
  for( int i = 0; i < Pk.size(); ++i ) {
    Eigen::Vector3d pt;
    pt = ( (Tf.block(0,0,3,3))*Pk[i] + Tf.block(0,3,3,1) );
 
    std::cout << " Pw["<<i<<"]: "<< Pw[i].transpose() <<
      " , Tf(Pk[i]): "<< pt.transpose() <<". Error: "<< (Pw[i] - pt).norm() << std::endl;
  }

  
}

