/**
 * @file tabletop_symmetry_test
 * @brief Test with hard-coded segmented pointcloud
 */

#include <tabletop_symmetry/mindGapper.h>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>

void help( char* argv0 ) {
  std::cout << " Help"<< std::endl;
  std::cout << "Usage: \t "<<argv0<< "-i CANDIDATE_INDEX -c CLOUD_INDEX"<<std::endl;
}

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  int candidate_index = 0;
  int cloud_index = 0;
  int c;

  while( (c = getopt(argc, argv,"i:c:h")) != -1 ) {
  
    switch(c) {
    case 'i':
      candidate_index = atoi(optarg);
      break;
    case 'c':
      cloud_index = atoi(optarg);
      break;
    case 'h':
    case '?':
      help( argv[0] );
      return 0;
    }

  }


  std::cout<<"\t -- Setting candidate index: "<<candidate_index<<" and cloud index: "<< cloud_index <<std::endl;
  

  mindGapper mG;
  std::vector<double> coeffs(4);
  coeffs[0] =  0.0379817; 
  coeffs[1] = -0.826591;
  coeffs[2] = -0.561521;
  coeffs[3] =  0.472441;
  
  // Set plane
  mG.setPlane( coeffs );
  
  // Set parameters for the optimization search of the best plane
  mG.setParams( 6, 5, 0.01, M_PI / 9.0 );

  // Convert (Debug version fires up a 
  char name[100];
  sprintf( name, "/home/ana/Research/toolsLWA4/tabletop_object_perception/bin/testSim/cluster_%d.pcd",
	   cloud_index );
  std::string filename( name );

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  
  if( pcl::io::loadPCDFile<pcl::PointXYZ>( filename.c_str(), *cloud ) == -1 ) {
    std::cout << "\t -- Did not load correctly point cloud "<< filename << std::endl;
    return 1;
  }

  if( !mG.complete( cloud ) ) { std::cout<< " FALSE!!!!" << std::endl; }
  mG.viewInitialParameters();

  mG.viewMirror( candidate_index );

  // Display
  int key;
  cv::Mat mask = mG.get2DMask();
  imwrite( "originalMask.png", mask );
  cv::imshow( "2D Segmented Mask", mask );
  while(true) {
    key = cv::waitKey(30);
    if( key != -1) {
      break;
    }
  }


  return 0;
}
