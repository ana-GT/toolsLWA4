#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/StdVector>

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > pk;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > pr;
int n = 12;

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  pk.resize(n);
  pr.resize(n);

  pk[0] << 0.039329, 0.204206, 0.871000;
  pr[0] << -0.161206, 0.509984, 0.548732;

  pk[1] << -0.086799, 0.118684, 1.020000;
  pr[1] << -0.271806, 0.379802, 0.422296;

  pk[2] << -0.072735, -0.045671, 0.974000;
  pr[2] << -0.271774, 0.226677, 0.536259;

  pk[3] << -0.083862, -0.208680, 1.123000;
  pr[3] << -0.272123, -0.000037, 0.472700;

  pk[4] << 0.044415, 0.094160, 1.023000;
  pr[4] << -0.130660, 0.334796, 0.468255;

  pk[5] << 0.206564, 0.165593, 0.983000;
  pr[5] << 0.024201, 0.415507, 0.513811;

  pk[6] << 0.301923, 0.052374, 0.887000;
  pr[6] << 0.100840, 0.341369, 0.659931;

  pk[7] << 0.177088, 0.074055, 0.927000;
  pr[7] << -0.015906, 0.348108, 0.594523;

  pk[8] << 0.192206, -0.095091, 1.165000;
  pr[8]<< 0.040345, 0.103321, 0.452210;
  
  pk[9] << 0.200932, -0.076354, 1.157000;
  pr[9] << 0.050239, 0.124578, 0.451264;

  pk[10] << 0.199734, 0.077256, 1.085000;
  pr[10] << 0.045849, 0.310440, 0.446621;

  pk[11] << 0.183340, 0.272973, 1.173000;
  pr[11] << 0.050178, 0.468402, 0.288904;

  // Fill in the CloudIn data
  cloud_in->width    = n;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);

  cloud_out->width    = n;
  cloud_out->height   = 1;
  cloud_out->is_dense = false;
  cloud_out->points.resize (cloud_out->width * cloud_out->height);


  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = pk[i](0);
    cloud_in->points[i].y = pk[i](1);
    cloud_in->points[i].z = pk[i](2);

    cloud_out->points[i].x = pr[i](0);
    cloud_out->points[i].y = pr[i](1);
    cloud_out->points[i].z = pr[i](2);

  }



  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  Eigen::Matrix4f tf = icp.getFinalTransformation();
  Eigen::Matrix4d Tf;
  for( int i = 0; i < 4; i++ ) {
    for( int j = 0; j < 4; j++ ) {
      Tf(i,j) = (double) tf(i,j);
    }
  }

  for( int i = 0; i < n; ++i ) {
    Eigen::Vector3d pt;
    pt = ( (Tf.block(0,0,3,3))*pk[i] + Tf.block(0,3,3,1) );
    std::cout << "[DEBUG] Orig point: "<< pr[i].transpose() <<
      " , Tf: "<< pt.transpose() <<" error: "<< (pr[i] - pt).norm() << std::endl;

  }




 return (0);
}

