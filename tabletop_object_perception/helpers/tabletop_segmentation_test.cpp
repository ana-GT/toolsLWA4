/**
 * @file tabletop_segmentation_test.cpp
 */
#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <tabletop_object_detector/tabletop_segmentation.h>


//-- Global variables
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gCloudPtr( new pcl::PointCloud<pcl::PointXYZRGBA> );
pcl::PointCloud<pcl::PointXYZ>::Ptr gFallbackCloudPtr( new pcl::PointCloud<pcl::PointXYZ> );
boost::shared_ptr<pcl::visualization::CloudViewer> gViewer;

pcl::Grabber* gKinectGrabber;
unsigned int gFilesSaved = 0;
bool gProcessCloud = false;
bool gNoColour = false;

TabletopSegmentor gTs;

//-- Functions declaration
void printUsage();
void grabberCallback( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &_cloud );
void keyboardEventOccurred( const pcl::visualization::KeyboardEvent &_event,
			    void *_nothing );
boost::shared_ptr<pcl::visualization::CloudViewer> createViewer();

/**
 * @function main
 * @brief Main function what else?
 */
int main( int argc, char* argv[] ) {

  // If user requires help
  if( pcl::console::find_argument( argc, argv, "-h" ) >= 0 ) {
    printUsage();
    return 0;
  }

  // If user wants to visualize a PCD
  bool justVisualize = false;
  std::string filename;

  if( pcl::console::find_argument( argc, argv, "-v" ) >= 0 ) {
    if( argc != 3 ) {
      printUsage( );
      return 0;
    }
    filename = argv[2];
    justVisualize = true;
  }

  // Otherwise, user is crazy 
  else if( argc != 1 ) {
    printUsage();
    return 0;
  }

  // If user indicated visualization of a video file
  if( justVisualize ) {
    try {
      pcl::io::loadPCDFile<pcl::PointXYZRGBA>( filename.c_str(), *gCloudPtr );
    }
    catch( pcl::PCLException e1 ) {
      try {
	pcl::io::loadPCDFile<pcl::PointXYZ>( filename.c_str(), *gFallbackCloudPtr );
      }
      catch( pcl::PCLException e2 ) {
	return -1;
      }
      gNoColour = true;
    }

    std::cout << "Loaded " << filename << "." << std::endl;
    if( gNoColour ) {
      std::cout << "This file has no RGBA colour information present" << std::endl;
    }

  } // end if

  // If user wants to use Kinect data
  else {
    gKinectGrabber = new pcl::OpenNIGrabber();
    if( gKinectGrabber == 0 ) {
      return false;
    }
    boost::function< void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > f = boost::bind( &grabberCallback, _1 );
    gKinectGrabber->registerCallback(f);


  }

  // Create viewer
  gViewer = createViewer();
  
  // Visualize either file data
  if( justVisualize ) {
    if( gNoColour ) {
      gViewer->showCloud( gFallbackCloudPtr );
    } else {
      gViewer->showCloud( gCloudPtr );
    }    
  }

  // Or visualize Kinect data
  else {
    gKinectGrabber->start();
  }

  while( !gViewer->wasStopped() ) {
    boost::this_thread::sleep( boost::posix_time::seconds(1) );
  }

  // If capturing Kinect data, stop the stream before exiting
  if( !justVisualize ) {
    gKinectGrabber->stop();
  }
  
}

///////////////// FUNCTION DEFINITIONS ////////////////////

/**
 * @function printUsage
 */
void printUsage( ) {
  printf( "**  Usage: \n" );
  printf( "<none> Start capturing from a Kinect device \n" );
  printf( "-v NAME : visualize the given .pcd file \n" );
  printf( "-h : Show this help");
  printf( "NOTE: To save a .pcd, press SPACE \n" );
}

/**
 * @function grabberCallback
 */
void grabberCallback( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &_cloud ) {

  if( !gViewer->wasStopped() ) {
    gViewer->showCloud( _cloud );
  }

  if( gProcessCloud ) {
    
    std::stringstream stream;
    stream << "inputCloud" << gFilesSaved << ".pcd";
    std::string filename = stream.str();
    
    if( pcl::io::savePCDFile( filename, *_cloud, true ) == 0 ) {
      gFilesSaved++;
      std::cout << "Saved " << filename << "." << std::endl;
    }
    else {
      PCL_ERROR( "[!]Problem saving %s. \n", filename.c_str() );
    }

    /** Process cloud */
    gTs.processCloud( _cloud );

    // Visualize camera parameters
    double fx, fy, cx, cy;
    fx = 0; fy = 0; cx = 0; cy = 0;
    std::cout << " BEFORE: Focal length x,y: "<< fx<<", "<< fy<<" principal point x,y: "<<cx<<", "<<cy<< std::endl;

    ((pcl::OpenNIGrabber*)gKinectGrabber)->getDepthCameraIntrinsics( fx, fy, cx, cy );
    std::cout << " Focal length x,y: "<< fx<<", "<< fy<<" principal point x,y: "<<cx<<", "<<cy<< std::endl;



    gProcessCloud = false;
  }

}

/**
 * @function keyboardEventOccurred
 */
void keyboardEventOccurred( const pcl::visualization::KeyboardEvent &_event,
			   void *_nothing ) {
  
  // Save cloud when pressing space
  if( _event.getKeySym() == "space" && _event.keyDown() ) {
    gProcessCloud = true;
  }
}

/**
 * @function createViewer
 */
boost::shared_ptr<pcl::visualization::CloudViewer> createViewer() {
  boost::shared_ptr<pcl::visualization::CloudViewer> v( new pcl::visualization::CloudViewer("3D Viewer") );
  v->registerKeyboardCallback( keyboardEventOccurred );

  return (v);
}
