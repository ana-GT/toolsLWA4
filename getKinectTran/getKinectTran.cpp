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

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI );
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

  printf( "\t * Clicked in (%d, %d) with kinect coords: (%f, %f, %f) and robot coords: (%f, %f, %f)\n", x, y, pk(0), pk(1), pk(2), pw(0), pw(1), pw(2) );

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

  int numMatches = Pk.size();

  if( numMatches < 4 ) {
    printf("\t * [ERROR] You need at least 4 points to get the Twk!\n");
    return;
  }

  // 1. Compute the weighted centroids of both kinect and world sets
  // (all points same weight in our case)
  Eigen::Vector3d xm; xm << 0, 0, 0; // Points from Kinect
  Eigen::Vector3d ym; ym << 0, 0, 0; // Points from kinematics

  for( int i = 0; i < numMatches; ++i ) {    
    xm += Pk[i];
    ym += Pw[i];
  }

  std::cout << "Bef, xm: "<< xm.transpose() << " ym: "<< ym.transpose() << std::endl;
  xm = xm / numMatches; ym = ym / numMatches;
  std::cout << "xm: "<< xm.transpose() << " ym: "<< ym.transpose() << std::endl;

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

  std::cout << "\t * Rotation calculated: \n"<< Rot << std::endl;
  std::cout << "\t * Translation calculated: \n"<< trans << std::endl;

  for( int i = 0; i < numMatches; ++i ) {
    
    std::cout << "[DEBUG] Orig point: "<< Pw[i].transpose() <<
      " and with Tf: "<< ( Rot*Pk[i] + trans ).transpose() << std::endl;

  }


}
