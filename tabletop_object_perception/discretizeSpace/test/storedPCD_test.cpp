/**
 * @file storedPCD_test.cpp
 */
#include <pcl/io/pcd_io.h>
#include <PCL2Octomap.h>
#include <octomap/ColorOcTree.h>
#include <tabletop_object_detector/tabletop_segmentation.h>

octomap::ColorOcTreeNode::Color gColor[10] = { octomap::ColorOcTreeNode::Color(255,0,0),
					       octomap::ColorOcTreeNode::Color(0, 255,0),
					       octomap::ColorOcTreeNode::Color(0,0,255),
					       octomap::ColorOcTreeNode::Color(255,255,0),
					       octomap::ColorOcTreeNode::Color(255,0,255),
					       octomap::ColorOcTreeNode::Color(0,255,255),
					       octomap::ColorOcTreeNode::Color(125,0,50),
					       octomap::ColorOcTreeNode::Color(0,50,100),
					       octomap::ColorOcTreeNode::Color(0,120,120),
					       octomap::ColorOcTreeNode::Color(50,0,200)}; 

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    if( argc < 2 ) {
	std::cout <<"Enter pcd to show" << std::endl;
	return 1;
    }
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGBA> );
    if( pcl::io::loadPCDFile<pcl::PointXYZRGBA>( argv[1], *cloud ) == -1 ) {
	std::cout<<"Could not read file "<< std::endl;
	return 1;
    }  

    // Segment tabletop
    TabletopSegmentor tts;
    tts.processCloud( cloud );

   
    octomap::Pointcloud octomapCloud;
    pointcloudPCLToOctomap( *cloud, 
			    octomapCloud );
    

    // Store in an octree
    double res = 0.02;
    octomap::ColorOcTree tree( res );
    octomap::ColorOcTreeNode* node;

    pcl::PointCloud<pcl::PointXYZRGBA> cluster;
    pcl::PointCloud<pcl::PointXYZRGBA>::iterator it;

    // Objects
    for( int i = 0; i < tts.getNumClusters(); ++i ) {
	cluster = tts.getCluster(i);
	for( it = cluster.begin(); it != cluster.end(); ++it ) {
	    node = tree.updateNode( octomap::point3d( it->x, it->y, it->z ), true );
	    if( node ) {
		node->setColor( gColor[i] );
	    }
	}
	
    }

    // Table
    cluster = tts.getTable();
    int ind = tts.getNumClusters();

    for( it = cluster.begin(); it != cluster.end(); ++it ) {
	node = tree.updateNode( octomap::point3d( it->x, it->y, it->z ), true );
	if( node ) {
	    node->setColor( gColor[ind] );
	}
    }
    

    // Write result

    std::cout <<"Done conversion!"<< std::endl;
    tree.write("tree.ot");
    

    return 0;
}



