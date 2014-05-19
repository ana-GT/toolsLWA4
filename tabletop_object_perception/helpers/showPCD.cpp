/**
 * @file showPCD.cpp
 */
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

//-- Global variables
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gCloudPtr( new pcl::PointCloud<pcl::PointXYZRGBA> );
boost::shared_ptr<pcl::visualization::PCLVisualizer> gViewer;
std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > gPointclouds;

const int MAX_PCD = 10;
double gColors[MAX_PCD][3];

//-- Functions declaration
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbViewer( std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clouds );
bool readPCDData( std::vector<std::string> _filenames, 
		  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &_pcdData );
void initColors();

/**
 * @function main
 * @brief Main function what else?
 */
int main( int argc, char* argv[] ) {

  std::vector< std::string > filenames;
  
  initColors();

  if( argc < 2 ) {
    printf( "Enter pcd to show \n" );
    return 0;
  }

  for( int i = 1; i < argc; ++i ) {
    std::string filename;
    filename = std::string( argv[i] );
    filenames.push_back( filename );
  }

  
  // Read  
  if( !readPCDData( filenames, gPointclouds ) ) {
    printf( " [X] Error reading PCD text file exiting \n" );
  }


  // Create viewer
  gViewer = rgbViewer( gPointclouds );
  

  while( !gViewer->wasStopped() ) {
    gViewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::seconds(1) );
  }

  
}

///////////////// FUNCTION DEFINITIONS ////////////////////

/**
 * @function createViewer
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbViewer ( std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clouds) {

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  
  for( int i = 0; i < clouds.size(); ++i ) {
    
    char name[50];
    sprintf( name, "cloud_%d", i );
    viewer->addPointCloud<pcl::PointXYZRGBA> (clouds[i], name );
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 
					      gColors[i][0], gColors[i][1], gColors[i][2], name );

  }

  viewer->addCoordinateSystem (1.0, 0);
  viewer->initCameraParameters ();
  return (viewer);
}


/**
 * @function readPCDData
 */
bool readPCDData( std::vector<std::string> _filenames, 
		  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &_clouds ) {

  for( int i = 0; i < _filenames.size(); ++i ) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGBA> );
    if( pcl::io::loadPCDFile<pcl::PointXYZRGBA>( _filenames[i].c_str(), *cloud ) == -1 ) {
      PCL_ERROR("Could not read file \n");
      return false;
    }
    _clouds.push_back( cloud );
  }

  return true;
}


/**
 * @function initColors
 */
void initColors() {

  gColors[0][0] = 1.0; gColors[0][1] = 0.0; gColors[0][2] = 0.0; // Red
  gColors[1][0] = 0.0; gColors[1][1] = 1.0; gColors[1][2] = 0.0; // Green
  gColors[2][0] = 0.0; gColors[2][1] = 0.0; gColors[2][2] = 1.0; // Blue
  gColors[3][0] = 1.0; gColors[3][1] = 0.0; gColors[3][2] = 1.0; // Magenta

  gColors[4][0] = 1.0; gColors[4][1] = 1.0; gColors[4][2] = 0.0; // Yellow
  gColors[5][0] = 0.0; gColors[5][1] = 1.0; gColors[5][2] = 1.0; // Cyan
  gColors[6][0] = 0.0; gColors[6][1] = 0.0; gColors[6][2] = 0.0; // White
  gColors[7][0] = 1.0; gColors[7][1] = 0.6; gColors[7][2] = 0.0; // Orange


  gColors[8][0] = 1.0; gColors[8][1] = 0.45; gColors[8][2] = 0.7; // Deep Pink
  gColors[9][0] = 0.6; gColors[9][1] = 0.4; gColors[9][2] = 0.9; // Violet

  
}
