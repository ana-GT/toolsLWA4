/**
 * @file tabletop_segmentation.h
 */
#pragma once

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>


//#include "tabletop_object_detector/TabletopSegmentation.h"

/**
 * @class TabletopSegmentor
 */
class TabletopSegmentor {
  
    typedef pcl::PointXYZRGBA    Point;
    typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;
    
public:

  /** Constructor */
  TabletopSegmentor(); 

  /** Destructor */
  ~TabletopSegmentor() {}


  //------------------- Complete processing -----

  //! Complete processing for new style point cloud
  /*void processCloud(const sensor_msgs::PointCloud2 &cloud,
                    TabletopSegmentation::Response &response, 
                    Table table); */

  void processCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &_cloud );
  
  int getNumClusters() { return mClusters.size(); }
  pcl::PointCloud<Point> getCluster( int _ind ) { 
      return mClusters[_ind];	  
  }
  pcl::PointCloud<Point> getTable() { 
      return mTable_Points;	  
  }


  //! Clears old published markers and remembers the current number of published markers
  /*void clearOldMarkers(std::string frame_id);*/

  //! Pull out and transform the convex hull points from a Table message
  /*
  template <class PointCloudType>
  bool tableMsgToPointCloud (Table &table, std::string frame_id, PointCloudType &table_hull);
  */
    
private:


  //! Min number of inliers for reliable plane detection
  int inlier_threshold_;
  //! Size of downsampling grid before performing plane detection
  double plane_detection_voxel_size_;
  //! Size of downsampling grid before performing clustering
  double clustering_voxel_size_;
  //! Filtering of original point cloud along the z, y, and x axes
  double z_filter_min_, z_filter_max_;
  double y_filter_min_, y_filter_max_;
  double x_filter_min_, x_filter_max_;
  //! Filtering of point cloud in table frame after table detection
  double table_z_filter_min_, table_z_filter_max_;
  //! Min distance between two clusters
  double cluster_distance_;
  //! Min number of points for a cluster
  int min_cluster_size_;
  //! Clouds are transformed into this frame before processing; leave empty if clouds
  //! are to be processed in their original frame
  std::string processing_frame_;
  //! Positive or negative z is closer to the "up" direction in the processing frame?
  double up_direction_;
  bool flatten_table_;
  //! How much the table gets padded in the horizontal direction
  double table_padding_;

  /** Info that will be messages */
  std::vector< pcl::PointCloud<Point> > mClusters;
  pcl::PointCloud<Point> mTable_Points;
  pcl::PointCloud<Point> mTableHull_Points;

  /** Debugging variables */
  pcl::PointCloud<Point> dDownsampledFilteredCloud;
  pcl::PointCloud<Point> dTableInliers;
  pcl::PointCloud<Point> dTableProjected;

  //------------------ Callbacks -------------------

  //! Callback for service calls
  /*bool serviceCallback( TabletopSegmentation::Request &request, 
    TabletopSegmentation::Response &response); */

  //------------------ Individual processing steps -------

  //! Converts raw table detection results into a Table message type
  /*template <class PointCloudType>
  Table getTable(std_msgs::Header cloud_header, const tf::Transform &table_plane_trans,
  const PointCloudType &table_points); */

  //! Converts table convex hull into a triangle mesh to add to a Table message
  /*template <class PointCloudType>
    void addConvexHullTable(Table &table, const PointCloudType &convex_hull, bool flatten_table);*/

  //! Publishes rviz markers for the given tabletop clusters
  /*template <class PointCloudType>
    void publishClusterMarkers(const std::vector<PointCloudType> &clusters, std_msgs::Header cloud_header);*/
  

};
