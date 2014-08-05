/**
 * @file tabletop_segmentation.cpp  
 * @brief Based on code from Marius Muja and Matei Ciocarlie
 */

#include "tabletop_segmentation.h"

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>


// NOTES
// READ WHAT RADU SAID!
//http://www.pcl-users.org/Point-Cloud-processing-in-grabber-callback-td3928466.html

/*************** HELPERS ***********************/

/**
 * @function getClustersFromPointCloud2
 */
template <typename PointT> 
void getClustersFromPointCloud2( const pcl::PointCloud<PointT> &_cloud_objects, 	    
				 const std::vector<pcl::PointIndices> &_clusters2, 
				 std::vector<pcl::PointCloud<PointT> > &_clusters ) {
  
  // Resize
  _clusters.resize( _clusters2.size() );
  
  // Fill the cluster with the indices
  for (size_t i = 0; i < _clusters2.size (); ++i) {
    
    pcl::PointCloud<PointT> cloud_cluster;
    pcl::copyPointCloud( _cloud_objects, _clusters2[i], cloud_cluster );
    _clusters[i] = cloud_cluster;
  }
}


/**
 * @function getPlaneTransform
 * @brief Assumes plane coefficients are of the form ax+by+cz+d=0, normalized 
 */    
Eigen::Matrix4d getPlaneTransform ( pcl::ModelCoefficients coeffs, 
				    double up_direction, 
				    bool flatten_plane ) {

  Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();

  if( coeffs.values.size() <= 3 ) {
    std::cout << "[ERROR] Coefficient size less than 3. I will output nonsense values"<<std::endl;
    return tf;
  }
  
  double a = coeffs.values[0]; 
  double b = coeffs.values[1]; 
  double c = coeffs.values[2];
  double d = coeffs.values[3];

  //asume plane coefficients are normalized
  Eigen::Vector3d position(-a*d, -b*d, -c*d);
  Eigen::Vector3d z(a, b, c);

  //if we are flattening the plane, make z just be (0,0,up_direction)
  if(flatten_plane) {

    std::cout <<"[INFO] Flattening plane"<<std::endl;
    z << 0, 0, up_direction;
  }

  else {
    //make sure z points "up"
    std::cout <<" [INFO] In getPlaneTransform, z: "<< z.transpose() <<std::endl;
    if ( z.dot( Eigen::Vector3d(0, 0, up_direction) ) < 0) {
      z = -1.0 * z;
      std::cout <<" \t [INFO] Flipped z" << std::endl;
    }
  }
  
  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  Eigen::Vector3d x; x << 1, 0, 0;
  if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = Eigen::Vector3d(0, 1, 0);
  Eigen::Vector3d y = z.cross(x); y.normalized();
  x = y.cross(z); x.normalized();

  std::cout << "Check:"<<std::endl;
  Eigen::Matrix3d rotation;
  rotation.block(0,0,3,1) = x; 
  rotation.block(0,1,3,1) = y;
  rotation.block(0,2,3,1) = z;

  Eigen::Quaterniond orientation( rotation );
  std::cout << "\t [DEBUG] In getPlaneTransform. Rotation matrix : \n"<< rotation << std::endl;

  tf.block(0,0,3,3) = orientation.matrix();
  tf.block(0,3,3,1) = position;

  return tf;

}



/**
 * @function getPlanePoints
 */      
template <typename PointT> 
bool getPlanePoints (const pcl::PointCloud<PointT> &table, 
		     const Eigen::Matrix4d& table_plane_trans,
		     pcl::PointCloud<PointT> &table_points ) {

  // Prepare the output      
  table_points.points.resize (table.points.size ());
  for (size_t i = 0; i < table.points.size (); ++i) {
    table_points.points[i].x = table.points[i].x;
    table_points.points[i].y = table.points[i].y;
    table_points.points[i].z = table.points[i].z;
  }

  /*
  // Transform the data
  tf::TransformListener listener;
  tf::StampedTransform table_pose_frame(table_plane_trans, table_points.header.stamp, 
                                        table_points.header.frame_id, "table_frame");
  listener.setTransform(table_pose_frame);
  std::string error_msg;
  if (!listener.canTransform("table_frame", table_points.header.frame_id, table_points.header.stamp, &error_msg))
  {
    ROS_ERROR("Can not transform point cloud from frame %s to table frame; error %s", 
	      table_points.header.frame_id.c_str(), error_msg.c_str());
    return false;
  }
  int current_try=0, max_tries = 3;
  while (1)
  {
    bool transform_success = true;
    try
    {
      listener.transformPointCloud("table_frame", table_points, table_points);
    }
    catch (tf::TransformException ex)
    {
      transform_success = false;
      if ( ++current_try >= max_tries )
      {
        ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s", 
                  table_points.header.frame_id.c_str(), ex.what());
        return false;
      }
      //sleep a bit to give the listener a chance to get a new transform
      ros::Duration(0.1).sleep();
    }
    if (transform_success) break;
  }
  table_points.header.frame_id = "table_frame";
  */
  return true;
}

 
/**
 * @function straightenPoints
 */	
template <class PointCloudType>
void straightenPoints( PointCloudType &points, 
		       const Eigen::Matrix4d& table_plane_trans, 
		       const Eigen::Matrix4d& table_plane_trans_flat ) {
  Eigen::Matrix4d trans = table_plane_trans_flat * table_plane_trans.inverse();
  pcl::transformPointCloud(points, points, trans);
}
	




/*********** CLASS FUNCTIONS ***************/

/**
 * @function TabletopSegmentor
 * @brief Constructor
 */
TabletopSegmentor::TabletopSegmentor() {

  
  //initialize operational flags
  inlier_threshold_ = 300;
  plane_detection_voxel_size_ = 0.01;
  
  clustering_voxel_size_ = 0.003;
  z_filter_min_ = 0.4;
  z_filter_max_ = 1.25;
  y_filter_min_ = -1.0;
  y_filter_max_ = 1.0;
  x_filter_min_ = -1.0;
  x_filter_max_ = 1.0;
  table_z_filter_min_= 0.01;
  table_z_filter_max_= 0.50;
  cluster_distance_ = 0.03;
  min_cluster_size_ = 300;
  processing_frame_ = "";
  up_direction_ = -1.0;   
  flatten_table_ = false;   
  table_padding_ = 0.0;   
  if(flatten_table_) { std::cout <<"\t ** Flatten_table is true" << std::endl; }
  else { std::cout <<"\t ** Flatten_table is false" << std::endl; }

  
}

/**
 * @function processCloud
 */
/*void TabletopSegmentor::processCloud(const sensor_msgs::PointCloud2 &cloud,
  TabletopSegmentation::Response &response,
  Table input_table) {*/

void TabletopSegmentor::processCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &_cloud ) {

  std::cout <<"[INFO] Start processing cloud" << std::endl;


  // PCL objects  
  KdTreePtr normals_tree_, clusters_tree_;
  pcl::VoxelGrid<Point> grid_, grid_objects_;
  pcl::PassThrough<Point> pass_;
  pcl::NormalEstimation<Point, pcl::Normal> n3d_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
  pcl::ProjectInliers<Point> proj_;
  pcl::ConvexHull<Point> hull_;
  pcl::ExtractPolygonalPrismData<Point> prism_;
  pcl::EuclideanClusterExtraction<Point> pcl_cluster_;
  pcl::PointCloud<Point>::Ptr table_hull_ptr (new pcl::PointCloud<Point>); 

  /************ STEP 0: Parameter Settings **************/
  
  // Filtering parameters
  grid_.setLeafSize (plane_detection_voxel_size_, plane_detection_voxel_size_, plane_detection_voxel_size_);
  grid_objects_.setLeafSize (clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);
  grid_.setFilterFieldName ("z");
  grid_.setFilterLimits (z_filter_min_, z_filter_max_);
  grid_.setDownsampleAllData (false);
  grid_objects_.setDownsampleAllData (true);

  normals_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();
  clusters_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();

  // Normal estimation parameters
  n3d_.setKSearch (10);  
  n3d_.setSearchMethod (normals_tree_);
  // Table model fitting parameters
  seg_.setDistanceThreshold (0.05); 
  seg_.setMaxIterations (10000);
  seg_.setNormalDistanceWeight (0.1);
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setProbability (0.99);

  proj_.setModelType (pcl::SACMODEL_PLANE);

  // Clustering parameters
  pcl_cluster_.setClusterTolerance (cluster_distance_);
  pcl_cluster_.setMinClusterSize (min_cluster_size_);
  pcl_cluster_.setSearchMethod (clusters_tree_);

  
  /******** Step 1 : Filter, remove NaNs and downsample ***********/


  // Filter in X, Y and Z directions: output= cloud_filtered_ptr
  pass_.setInputCloud (_cloud);
  pass_.setFilterFieldName ("z");
  pass_.setFilterLimits (z_filter_min_, z_filter_max_);
  pcl::PointCloud<Point>::Ptr z_cloud_filtered_ptr (new pcl::PointCloud<Point>); 
  pass_.filter (*z_cloud_filtered_ptr);

  pass_.setInputCloud (z_cloud_filtered_ptr);
  pass_.setFilterFieldName ("y");
  pass_.setFilterLimits (y_filter_min_, y_filter_max_);
  pcl::PointCloud<Point>::Ptr y_cloud_filtered_ptr (new pcl::PointCloud<Point>); 
  pass_.filter (*y_cloud_filtered_ptr);

  pass_.setInputCloud (y_cloud_filtered_ptr);
  pass_.setFilterFieldName ("x");
  pass_.setFilterLimits (x_filter_min_, x_filter_max_);
  pcl::PointCloud<Point>::Ptr cloud_filtered_ptr (new pcl::PointCloud<Point>); 
  pass_.filter (*cloud_filtered_ptr);

  // Check that the points filtered at least are of a minimum size
  if (cloud_filtered_ptr->points.size() < (unsigned int)min_cluster_size_) {
    std::cout <<"\t [BAD] Filtered cloud only has "<< (int)cloud_filtered_ptr->points.size() << " points."<<std::endl;
    //response.result = response.NO_TABLE;
    return;
  }

  // Downsample the filtered cloud: output = cloud_downsampled_ptr
  pcl::PointCloud<Point>::Ptr cloud_downsampled_ptr (new pcl::PointCloud<Point>); 
  grid_.setInputCloud (cloud_filtered_ptr);
  grid_.filter (*cloud_downsampled_ptr);
  if (cloud_downsampled_ptr->points.size() < (unsigned int)min_cluster_size_) {
    std::cout <<"\t [BAD] Downsampled cloud only has "<< (int)cloud_downsampled_ptr->points.size()<<" points."<<std::endl;
    //response.result = response.NO_TABLE;    
    return;
  } 
  
  std::cout<<"\t [OK] Step 1 done. Downsample filtered cloud has : "
	   <<( int)cloud_downsampled_ptr->points.size() 
	   <<" points."<<std::endl;
 
  // DEBUG ------------
  dDownsampledFilteredCloud = *cloud_downsampled_ptr;
  if( pcl::io::savePCDFile( "downsampledFilteredCloud.pcd", *cloud_downsampled_ptr, true ) == 0 ) {
    std::cout << "Saved DEBUG filtered downsampled cloud"<< std::endl;
  }
  // DEBUG ------------


  /***************** Step 2 : Estimate normals ******************/

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>); 
  n3d_.setInputCloud (cloud_downsampled_ptr);
  n3d_.compute (*cloud_normals_ptr);
  std::cout <<"\t [OK] Step 2 done"<<std::endl;
  

  /************* Step 3 : Perform planar segmentation, **********/
  /**     if table is not given, otherwise use given table      */  
  Eigen::Matrix4d table_plane_trans; 
  Eigen::Matrix4d table_plane_trans_flat;

  /*
  if(input_table.convex_hull.vertices.size() != 0) {  
    
    std::cout << "Table input, skipping Step 3" << std::endl;
    bool success = tableMsgToPointCloud<pcl::PointCloud<Point> >(input_table, cloud.header.frame_id, *table_hull_ptr);
    if(!success) {
      std::cout <<"[ERROR] Failure in converting table convex hull!" << std::endl;
      return;
    }
    

    //response.table = input_table;
    if(flatten_table_) {
      std::cout<<"flatten_table mode is disabled if table is given!" << std::endl;
      flatten_table_ = false;
    }
  }*/
  
  bool inputTableFlag = false;
  if( inputTableFlag == true ) { std::cout << "DO NOT CALL ME"<<std::endl; }
  else
  {
    pcl::PointIndices::Ptr table_inliers_ptr (new pcl::PointIndices); 
    pcl::ModelCoefficients::Ptr table_coefficients_ptr (new pcl::ModelCoefficients); 
    seg_.setInputCloud (cloud_downsampled_ptr);
    seg_.setInputNormals (cloud_normals_ptr);
    seg_.segment( *table_inliers_ptr, 
		  *table_coefficients_ptr );
    
    
    // Check the coefficients and inliers are above threshold values
    if (table_coefficients_ptr->values.size () <=3 ) {
      std::cout <<"\t [ERROR] Failed to detect table in scan" << std::endl;
      //response.result = response.NO_TABLE;    
      return;
    }
    
    if ( table_inliers_ptr->indices.size() < (unsigned int)inlier_threshold_) {
      std::cout <<"\t [ERROR] Plane detection has "<< (int)table_inliers_ptr->indices.size() <<
	" inliers, below min threshold of " << inlier_threshold_ << std::endl;
      //response.result = response.NO_TABLE;
      return;
    }
    
    std::cout <<"\t [OK] Table segmented  with " 
	      << (int)table_inliers_ptr->indices.size () 
	      << " inliers and coefficients: " 
	      << table_coefficients_ptr->values[0] <<" " 
	      << table_coefficients_ptr->values[1] <<" " 
	      << table_coefficients_ptr->values[2] <<" " 
	      << table_coefficients_ptr->values[3] << std::endl;
    
    // Store table's plane coefficients
    mTableCoeffs.resize(4);
    for( int i = 0; i < 4; ++i ) {
      mTableCoeffs[i] = table_coefficients_ptr->values[i];
    }

    // DEBUG ------------
    pcl::copyPointCloud( *cloud_downsampled_ptr, *table_inliers_ptr, dTableInliers );
    if( pcl::io::savePCDFile( "table_inliers.pcd", dTableInliers, true ) == 0 ) {
      std::cout << "\t [DEBUG] Saved DEBUG table inliers cloud"<< std::endl;
    }
    // DEBUG ------------

    std::cout<<"\t [OK] Step 3 done" << std::endl;
    
    
    /**********  Step 4 : Project the table inliers on the table *********/
    pcl::PointCloud<Point>::Ptr table_projected_ptr (new pcl::PointCloud<Point>); 
    proj_.setInputCloud (cloud_downsampled_ptr);
    proj_.setIndices (table_inliers_ptr);
    proj_.setModelCoefficients (table_coefficients_ptr);
    proj_.filter (*table_projected_ptr);
    std::cout <<"\t [OK] Step 4 done"<<std::endl;


    // DEBUG ------------
    dTableProjected = *table_projected_ptr;
    if( pcl::io::savePCDFile( "table_projected.pcd", dTableProjected, true ) == 0 ) {
      std::cout << "\t [DEBUG] Saved DEBUG table projected cloud"<< std::endl;
    }
    // DEBUG ------------  

    // Get the table transformation w.r.t. camera
    table_plane_trans = getPlaneTransform (*table_coefficients_ptr, up_direction_, false);

    // ---[ Estimate the convex hull (not in table frame)
    hull_.setInputCloud (table_projected_ptr);
    hull_.reconstruct (*table_hull_ptr);
  
    if(!flatten_table_)
    {
      // --- [ Take the points projected on the table and transform them into the PointCloud message
      //  while also transforming them into the table's coordinate system
      if (!getPlanePoints<Point> (*table_projected_ptr, table_plane_trans, mTable_Points)) {
        //response.result = response.OTHER_ERROR;
        return;
      }

      // ---[ Create the table message
      //response.table = getTable<sensor_msgs::PointCloud>(cloud.header, table_plane_trans, table_points);

      // ---[ Convert the convex hull points to table frame
      if (!getPlanePoints<Point> (*table_hull_ptr, table_plane_trans, mTableHull_Points))
      {
        //response.result = response.OTHER_ERROR;
        return;
      }
    }
    /*
    if(flatten_table_)
    {
      // if flattening the table, find the center of the convex hull and move the table frame there
      table_plane_trans_flat = getPlaneTransform (*table_coefficients_ptr, up_direction_, flatten_table_);
      tf::Vector3 flat_table_pos;
      double avg_x, avg_y, avg_z;
      avg_x = avg_y = avg_z = 0;
      for (size_t i=0; i<table_projected_ptr->points.size(); i++)
      {
        avg_x += table_projected_ptr->points[i].x;
        avg_y += table_projected_ptr->points[i].y;
        avg_z += table_projected_ptr->points[i].z;
      }
      avg_x /= table_projected_ptr->points.size();
      avg_y /= table_projected_ptr->points.size();
      avg_z /= table_projected_ptr->points.size();
      ROS_INFO("average x,y,z = (%.5f, %.5f, %.5f)", avg_x, avg_y, avg_z);

      // place the new table frame in the center of the convex hull
      flat_table_pos[0] = avg_x;
      flat_table_pos[1] = avg_y;
      flat_table_pos[2] = avg_z;
      table_plane_trans_flat.setOrigin(flat_table_pos);

      // shift the non-flat table frame to the center of the convex hull as well
      table_plane_trans.setOrigin(flat_table_pos);

      // --- [ Take the points projected on the table and transform them into the PointCloud message
      //  while also transforming them into the flat table's coordinate system
      sensor_msgs::PointCloud flat_table_points;
      if (!getPlanePoints<Point> (*table_projected_ptr, table_plane_trans_flat, flat_table_points))
      {
        response.result = response.OTHER_ERROR;
        return;
      }

      // ---[ Create the table message
      response.table = getTable<sensor_msgs::PointCloud>(cloud.header, table_plane_trans_flat, flat_table_points);

      // ---[ Convert the convex hull points to flat table frame
      if (!getPlanePoints<Point> (*table_hull_ptr, table_plane_trans_flat, table_hull_points))
      {
        response.result = response.OTHER_ERROR;
        return;
      }
    }

    ROS_INFO("Table computed");  
    // ---[ Add the convex hull as a triangle mesh to the Table message
    addConvexHullTable<sensor_msgs::PointCloud>(response.table, table_hull_points, flatten_table_);
  */
  } // ELSE END, APPARENTLY

  
  // Step 5: Get the objects on top of the (non-flat) table
  
  pcl::PointIndices cloud_object_indices;
  prism_.setInputCloud (cloud_filtered_ptr);
  prism_.setInputPlanarHull (table_hull_ptr);
  
  std::cout <<" \t [INFO] Using table prism: "<< table_z_filter_min_ << " to "<< table_z_filter_max_<<std::endl;
  prism_.setHeightLimits (table_z_filter_min_, table_z_filter_max_);  
  prism_.segment (cloud_object_indices);
  
  pcl::PointCloud<Point>::Ptr cloud_objects_ptr (new pcl::PointCloud<Point>); 
  pcl::ExtractIndices<Point> extract_object_indices;
  extract_object_indices.setInputCloud (cloud_filtered_ptr);
  extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
  extract_object_indices.filter (*cloud_objects_ptr);

  std::cout<<"\t [INFO] Number of object point candidates: " <<(int)cloud_objects_ptr->points.size() << std::endl;
  //response.result = response.SUCCESS;

  if (cloud_objects_ptr->points.empty ())  {
    std::cout<<"\t [ERROR] No objects on table" << std::endl;
    return;
  }
  
  //  Downsample the points
  pcl::PointCloud<Point>::Ptr cloud_objects_downsampled_ptr (new pcl::PointCloud<Point>); 
  grid_objects_.setInputCloud (cloud_objects_ptr);
  grid_objects_.filter (*cloud_objects_downsampled_ptr);

  // If flattening the table, adjust the points on the table to be straight also  
  if(flatten_table_) straightenPoints<pcl::PointCloud<Point> >(*cloud_objects_downsampled_ptr, 
  							       table_plane_trans, table_plane_trans_flat);
							       
  

  // Step 6: Split the objects into Euclidean clusters
  std::vector<pcl::PointIndices> clusters2;
  pcl_cluster_.setInputCloud (cloud_objects_downsampled_ptr);
  pcl_cluster_.extract (clusters2);
  std::cout<<"\t [OK] Number of clusters found matching the given constraints: "<<(int)clusters2.size () << std::endl;

  // Convert clusters into the PointCloud message
  getClustersFromPointCloud2<Point> (*cloud_objects_downsampled_ptr, clusters2, mClusters );
  std::cout<<"\t [OK] Clusters converted"<<std::endl;
  //response.clusters = clusters;  

  //publishClusterMarkers(clusters, cloud.header);


  // DEBUG ------------
  for( int i = 0; i < mClusters.size(); ++i ) {
    char name[50];
    sprintf(name, "cluster_%d.pcd", i );    
    if( pcl::io::savePCDFile( name, mClusters[i], true ) != 0 ) {
      std::cout << "\t [DEBUG ERROR] Error saving cluster cloud"<< std::endl;
    }
  }
  std::cout << "\t[DEBUG] Saved clusters all right"<< std::endl;
  // DEBUG ------------  

  

  std::cout << "Finished processing cloud" << std::endl;
}




/*! Processes the latest point cloud and gives back the resulting array of models.
 */
/*
bool TabletopSegmentor::serviceCallback(TabletopSegmentation::Request &request, 
                                        TabletopSegmentation::Response &response)
{
  ros::Time start_time = ros::Time::now();
  std::string topic = nh_.resolveName("cloud_in");
  ROS_INFO("Tabletop detection service called; waiting for a point_cloud2 on topic %s", topic.c_str());

  sensor_msgs::PointCloud2::ConstPtr recent_cloud = 
    ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_, ros::Duration(3.0));
              
  if (!recent_cloud)
  {
    ROS_ERROR("Tabletop object detector: no point_cloud2 has been received");
    response.result = response.NO_CLOUD_RECEIVED;
    return true;
  }

  pcl::PointCloud<Point>::Ptr table_hull (new pcl::PointCloud<Point>);
  ROS_INFO_STREAM("Point cloud received after " << ros::Time::now() - start_time << " seconds; processing");
  if (!processing_frame_.empty())
  {
    //convert cloud to processing_frame_ (usually base_link)
    sensor_msgs::PointCloud old_cloud;  
    sensor_msgs::convertPointCloud2ToPointCloud (*recent_cloud, old_cloud);
    int current_try=0, max_tries = 3;
    while (1)
    {
      bool transform_success = true;
      try
      {
        listener_.transformPointCloud(processing_frame_, old_cloud, old_cloud);    
      }
      catch (tf::TransformException ex)
      {
        transform_success = false;
        if (++current_try >= max_tries)
        {
          ROS_ERROR("Failed to transform cloud from frame %s into frame %s in %d attempt(s)", old_cloud.header.frame_id.c_str(), 
                    processing_frame_.c_str(), current_try);
          response.result = response.OTHER_ERROR;
          return true;
        }
        ROS_DEBUG("Failed to transform point cloud, attempt %d out of %d, exception: %s", current_try, max_tries, ex.what());
        //sleep a bit to give the listener a chance to get a new transform
        ros::Duration(0.1).sleep();
      }
      if (transform_success) break;
    }
    sensor_msgs::PointCloud2 converted_cloud;
    sensor_msgs::convertPointCloudToPointCloud2 (old_cloud, converted_cloud);
    ROS_INFO_STREAM("Input cloud converted to " << processing_frame_ << " frame after " <<
                    ros::Time::now() - start_time << " seconds");
    processCloud(converted_cloud, response, request.table);
    clearOldMarkers(converted_cloud.header.frame_id);
  }
  else
  {
    processCloud(*recent_cloud, response, request.table);
    clearOldMarkers(recent_cloud->header.frame_id);
  }

  //add the timestamp from the original cloud
  response.table.pose.header.stamp = recent_cloud->header.stamp;
  for(size_t i; i<response.clusters.size(); i++)
  {
    response.clusters[i].header.stamp = recent_cloud->header.stamp;
  }

  ROS_INFO_STREAM("In total, segmentation took " << ros::Time::now() - start_time << " seconds");
  return true;
}
*/

 /*
template <class PointCloudType>
void TabletopSegmentor::addConvexHullTable(Table &table,
					   const PointCloudType &convex_hull,
					   bool flatten_table)
{
  if (convex_hull.points.empty())
  {
    ROS_ERROR("Trying to add convex hull, but it contains no points");
    return;
  }
  //compute centroid
  geometry_msgs::Point centroid;
  centroid.x = centroid.y = centroid.z = 0.0;
  for (size_t i=0; i<convex_hull.points.size(); i++)
  {
    centroid.x += convex_hull.points[i].x;
    centroid.y += convex_hull.points[i].y;
    centroid.z += convex_hull.points[i].z;
  }
  centroid.x /= convex_hull.points.size();
  centroid.y /= convex_hull.points.size();
  centroid.z /= convex_hull.points.size();

  //create a triangle mesh out of the convex hull points and add it to the table message
  for (size_t i=0; i<convex_hull.points.size(); i++)
  {
    geometry_msgs::Point vertex;
    vertex.x = convex_hull.points[i].x;
    vertex.y = convex_hull.points[i].y;
    if (table_padding_ > 0.0)
    {
      double dx = vertex.x - centroid.x;
      double dy = vertex.y - centroid.y;
      double l = sqrt(dx*dx + dy*dy);
      dx /= l; dy /= l;
      vertex.x += table_padding_ * dx;
      vertex.y += table_padding_ * dy;
    }
    if(flatten_table) vertex.z = 0;
    else vertex.z = convex_hull.points[i].z;
    table.convex_hull.vertices.push_back(vertex);
      
    if(i==0 || i==convex_hull.points.size()-1) continue;
    shape_msgs::MeshTriangle meshtri;
    meshtri.vertex_indices[0] = 0;
    meshtri.vertex_indices[1] = i;
    meshtri.vertex_indices[2] = i+1;
    table.convex_hull.triangles.push_back(meshtri);
  }
  visualization_msgs::Marker tableMarker = MarkerGenerator::getConvexHullTableMarker(table.convex_hull);
  tableMarker.header = table.pose.header;
  tableMarker.pose = table.pose.pose;
  tableMarker.ns = "tabletop_node";
  tableMarker.id = current_marker_id_++;
  marker_pub_.publish(tableMarker);

  visualization_msgs::Marker originMarker = 
    MarkerGenerator::createMarker(table.pose.header.frame_id, 0, .0025, .0025, .01, 0, 1, 1, 
				  visualization_msgs::Marker::CUBE, current_marker_id_++, 
				  "tabletop_node", table.pose.pose);
  marker_pub_.publish(originMarker);
}
*/
  /*
template <class PointCloudType>
Table TabletopSegmentor::getTable(std_msgs::Header cloud_header,
                                  const tf::Transform &table_plane_trans, 
                                  const PointCloudType &table_points)
{
  Table table;
 
  //get the extents of the table
  if (!table_points.points.empty()) 
  {
    table.x_min = table_points.points[0].x;
    table.x_max = table_points.points[0].x;
    table.y_min = table_points.points[0].y;
    table.y_max = table_points.points[0].y;
  }  
  for (size_t i=1; i<table_points.points.size(); ++i) 
  {
    if (table_points.points[i].x<table.x_min && table_points.points[i].x>-3.0) table.x_min = table_points.points[i].x;
    if (table_points.points[i].x>table.x_max && table_points.points[i].x< 3.0) table.x_max = table_points.points[i].x;
    if (table_points.points[i].y<table.y_min && table_points.points[i].y>-3.0) table.y_min = table_points.points[i].y;
    if (table_points.points[i].y>table.y_max && table_points.points[i].y< 3.0) table.y_max = table_points.points[i].y;
  }

  geometry_msgs::Pose table_pose;
  tf::poseTFToMsg(table_plane_trans, table_pose);
  table.pose.pose = table_pose;
  table.pose.header = cloud_header;

  
  visualization_msgs::Marker tableMarker = MarkerGenerator::getTableMarker(table.x_min, table.x_max,
                                                                           table.y_min, table.y_max);
  tableMarker.header = cloud_header;
  tableMarker.pose = table_pose;
  tableMarker.ns = "tabletop_node";
  tableMarker.id = current_marker_id_++;
  marker_pub_.publish(tableMarker);
  
  return table;
}
  */
   /*
template <class PointCloudType>
void TabletopSegmentor::publishClusterMarkers(const std::vector<PointCloudType> &clusters, std_msgs::Header cloud_header)
{
  for (size_t i=0; i<clusters.size(); i++) 
  {
    visualization_msgs::Marker cloud_marker =  MarkerGenerator::getCloudMarker(clusters[i]);
    cloud_marker.header = cloud_header;
    cloud_marker.pose.orientation.w = 1;
    cloud_marker.ns = "tabletop_node";
    cloud_marker.id = current_marker_id_++;
    marker_pub_.publish(cloud_marker);
  }
}
   */
    /*
void TabletopSegmentor::clearOldMarkers(std::string frame_id)
{
  for (int id=current_marker_id_; id < num_markers_published_; id++)
    {
      visualization_msgs::Marker delete_marker;
      delete_marker.header.stamp = ros::Time::now();
      delete_marker.header.frame_id = frame_id;
      delete_marker.id = id;
      delete_marker.action = visualization_msgs::Marker::DELETE;
      delete_marker.ns = "tabletop_node";
      marker_pub_.publish(delete_marker);
    }
  num_markers_published_ = current_marker_id_;
  current_marker_id_ = 0;
}
    */

      /*
template <class PointCloudType>
bool TabletopSegmentor::tableMsgToPointCloud (Table &table, std::string frame_id, PointCloudType &table_hull)
{
  //use table.pose (PoseStamped) to transform table.convex_hull.vertices (Point[]) into a pcl::PointCloud in frame_id
  ros::Time now = ros::Time::now();
  PointCloudType table_frame_points;
  table_frame_points.header.stamp = now.toNSec();
  table_frame_points.header.frame_id = "table_frame";
  table_frame_points.points.resize(table.convex_hull.vertices.size());
  for(size_t i=0; i < table.convex_hull.vertices.size(); i++)
  {
    table_frame_points.points[i].x = table.convex_hull.vertices[i].x;
    table_frame_points.points[i].y = table.convex_hull.vertices[i].y;
    table_frame_points.points[i].z = table.convex_hull.vertices[i].z;
  }

  //add the table frame transform to the tf listener
  tf::Transform table_trans;
  tf::poseMsgToTF(table.pose.pose, table_trans);
  tf::StampedTransform table_pose_frame(table_trans, now, table.pose.header.frame_id, "table_frame");
  listener_.setTransform(table_pose_frame);
  ROS_INFO("transforming point cloud from table frame to frame %s", frame_id.c_str());

  //make sure we can transform
  std::string error_msg;
  if (!listener_.waitForTransform(frame_id, "table_frame", now, ros::Duration(2.0), ros::Duration(0.01), &error_msg))
  {
    ROS_ERROR("Can not transform point cloud from table frame to frame %s; error %s", 
	      frame_id.c_str(), error_msg.c_str());
    return false;
  }

  //transform the points
  int current_try=0, max_tries = 3;
  while (1)
  {
    bool transform_success = true;
    try
    {
      pcl_ros::transformPointCloud<Point>(frame_id, table_frame_points, table_hull, listener_); 
    }
    catch (tf::TransformException ex)
    {
      transform_success = false;
      if ( ++current_try >= max_tries )
      {
        ROS_ERROR("Failed to transform point cloud from table frame into frame %s; error %s", 
                  frame_id.c_str(), ex.what());
        return false;
      }
      //sleep a bit to give the listener a chance to get a new transform
      ros::Duration(0.1).sleep();
    }
    if (transform_success) break;
  }
  table_hull.header.frame_id = frame_id;
  table_hull.header.stamp = now.toNSec();

  //copy the transformed points back into the Table message and set the pose to identity in the cloud frame
  for(size_t i=0; i < table.convex_hull.vertices.size(); i++)
  {
    table.convex_hull.vertices[i].x = table_hull.points[i].x;
    table.convex_hull.vertices[i].y = table_hull.points[i].y;
    table.convex_hull.vertices[i].z = table_hull.points[i].z;
  }
  geometry_msgs::PoseStamped iden;
  iden.pose.orientation.w = 1;
  table.pose = iden;
  table.pose.header.frame_id = frame_id;

  //make a new Shape for drawing
  shape_msgs::Mesh mesh;
  mesh.vertices.resize(table_hull.points.size());
  for(size_t i = 0; i < table_hull.points.size(); i++)
  {
    mesh.vertices[i].x = table_hull.points[i].x;
    mesh.vertices[i].y = table_hull.points[i].y;
    mesh.vertices[i].z = table_hull.points[i].z;
  }
  mesh.triangles = table.convex_hull.triangles;
  visualization_msgs::Marker tableMarker = MarkerGenerator::getConvexHullTableMarker(mesh);
  pcl_conversions::fromPCL(table_hull.header, tableMarker.header);
  //tableMarker.header = table_hull.header;
  tableMarker.pose.orientation.w = 1.0;
  tableMarker.ns = "tabletop_node";
  tableMarker.id = current_marker_id_++;
  marker_pub_.publish(tableMarker);

  return true;
}
*/


	 

	 
