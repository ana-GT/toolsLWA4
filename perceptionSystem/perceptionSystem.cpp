/**
 * @file perceptionSystem
 * @brief Captures a snapshot, segments and select an object to be picked up
 * @author A. Huaman Q.
 */
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <ach.h>
#include <sns.h>

#include <Eigen/Core>
#include <stdint.h>
#include "tabletop_segmentation/tabletop_segmentation.h"


std::string windowName = std::string("Robot View");
cv::Mat rgbImg;
cv::Mat pclMap;

ach_channel_t obj_pos_chan;
Eigen::Vector3d currentPoint;
bool isSegmentedFlag = false;
double f;

std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > clusters;
std::vector<cv::Vec3b> colors;
std::vector< std::vector<int> > pixelClustersX;
std::vector< std::vector<int> > pixelClustersY;

/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int flags, void* userdata );
void startComm( int state, void* userdata );
void process( int state, void* userdata );
void sendMsg( int state, void* userdata );

void drawSegmented();
void getPixelClusters();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Initialization
  srand( time(NULL) );

  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI );
  
  if( !capture.isOpened() ) {
    std::cout << "/t * Could not open the capture object"<<std::endl;
    return -1;
  }
  
  // Set control panel
  cv::namedWindow( windowName, cv::WINDOW_AUTOSIZE );

  
  cv::createButton( "Start comm", startComm, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );

  int value;
  cv::createTrackbar("track1", "Robot view", &value, 255, NULL, NULL );

  cv::createButton( "Process", process, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );


  cv::createButton( "Send", sendMsg, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  

  // Set mouse callback 
  cv::setMouseCallback( windowName, onMouse, 0 );

  // Loop
  f = (float)capture.get( cv::CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH );

  for(;;) {
    
    if( !capture.grab() ) {
      std::cout << "\t * [DAMN] Could not grab a frame" << std::endl;
      return -1;
    }
    
    capture.retrieve( rgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    if( isSegmentedFlag ) {
      drawSegmented();
    }
    cv::imshow( windowName, rgbImg );
         
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
  p = pclMap.at<cv::Point3f>(y,x);
  currentPoint << (double)p.x*-1, (double)p.y, (double)p.z;

  std::cout << "\t * [INFO] Current point ready to send: "<< currentPoint.transpose() << std::endl;
  
}

/**
 * @function startComm
 */
void startComm( int state, void* userdata ) {

  printf("\t * Start communication \n");
  sns_init();
  sns_start();

  sns_chan_open( &obj_pos_chan, "obj-pos", NULL );  
  printf("\t * [OK] Communication stablished and ready to go \n");

}


/**
 * @function process
 */
void process( int state, 
	      void* userdata ) {


  // Get organized pointcloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGBA> );
  cv::Point3f p;
  cv::Vec3i col;
  pcl::PointXYZRGBA P;

  int width = pclMap.cols;
  int height = pclMap.rows;

  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false; // some NaN can be found
  cloud->points.resize( width * height );

  for( size_t j = 0; j < height; ++j ) {
    for( size_t i = 0; i < width; ++i ) {

      p = pclMap.at<cv::Point3f>(j,i);
      P.x = p.x*-1; P.y = p.y; P.z = p.z;
      P.r = (uint8_t)rgbImg.at<cv::Vec3b>(j,i)[2];
      P.g = (uint8_t)rgbImg.at<cv::Vec3b>(j,i)[1]; 
      P.b = (uint8_t)rgbImg.at<cv::Vec3b>(j,i)[0];
      P.a = 255;
      cloud->points[width*j + i] = P;
      
    }
  }

  // Segment
  TabletopSegmentor<pcl::PointXYZRGBA> tts;

  tts.processCloud( cloud );
  int n = tts.getNumClusters();
  printf("\t * Num clusters: %d \n", n );
  pcl::io::savePCDFileASCII<pcl::PointXYZRGBA>("testPoint.pcd", *cloud);

  // Set segmented variables
  isSegmentedFlag = true;
  clusters.resize(n);
  colors.resize(n);
  
  for( int i = 0; i < n; ++i ) {
    clusters[i] = tts.getCluster(i);

    cv::Vec3b def;
    def(0) = rand() % 255; def(1) = rand() % 255; def(2) = rand() % 255;  
    colors[i] = def;
  }
  getPixelClusters();
  
}

/**
 * @function process
 */
void sendMsg( int state, void* userdata ) {

  // Send through channel
  double msg[3];
  for( int i =0; i < 3; ++i ) { msg[i] = currentPoint(i); }

  ach_status r;
  r = ach_put( &obj_pos_chan, msg, sizeof(msg) );
  if( r != ACH_OK ) {
    printf("\t * [BAD] Something bad happened while sending obj Pos \n");
  }

}

/**
 * @function drawSegmented
 */
void drawSegmented() {
  for( int i = 0; i < pixelClustersX.size(); ++i ) {
    for( int j = 0; j < pixelClustersX[i].size(); ++j ) {
      rgbImg.at<cv::Vec3b>( pixelClustersY[i][j], pixelClustersX[i][j] ) = colors[i];
    }
  }
  
}

/**
 * @function getPixelClusters
 */
void getPixelClusters() {

  pixelClustersX.resize( clusters.size() );
  pixelClustersY.resize( clusters.size() );
  for( int i = 0; i < pixelClustersX.size(); ++i ) {
    pixelClustersX[i].resize(0);
    pixelClustersY[i].resize(0);
  }

  int u, v;
  int width, height;
  double X, Y, Z; 

  // Get (u,v) pixel of clusters  
  width = rgbImg.cols;
  height = rgbImg.rows;

  for( int i = 0; i < clusters.size(); ++i ) {
    for( int j = 0; j < clusters[i].points.size(); ++j ) {

      X = clusters[i].points[j].x;
      Y = clusters[i].points[j].y;
      Z = clusters[i].points[j].z;

      u = width/2 - (int)(X*f/Z);
      v = height/2 -(int)(Y*f/Z);

      pixelClustersX[i].push_back(u);
      pixelClustersY[i].push_back(v);
    }
  }

  
}
