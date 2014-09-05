#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/StdVector>

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pk;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pr;
int n = 14;

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  Pk.resize(n);
  Pr.resize(n);


Pk[0] << 0.176150, 0.313715, 0.966000; Pr[0] << 0.050160, 0.468393, 0.288904;
Pk[1] << 0.170304, 0.110383, 0.908000; Pr[1] << 0.045830, 0.310416, 0.446611;
Pk[2] << 0.174337, -0.028176, 1.014000; Pr[2] << 0.054553, 0.126199, 0.432561;
Pk[3] << 0.164567, -0.046508, 1.030000; Pr[3] << 0.044347, 0.105062, 0.433419;
Pk[4] << 0.121617, 0.081078, 0.753000; Pr[4] << -0.015849, 0.348099, 0.594581;
Pk[5] << 0.236176, 0.043827, 0.701000; Pr[5] << 0.100861, 0.341373, 0.659949;
Pk[6] << 0.168325, 0.173799, 0.788000; Pr[6] << 0.024200, 0.415512, 0.513829;
Pk[7] << 0.007563, 0.127061, 0.871000; Pr[7] << -0.130705, 0.334754, 0.468285;
Pk[8] << -0.127372, -0.150694, 1.033000; Pr[8] << -0.272122, -0.000054, 0.472733;
Pk[9] << -0.128743, -0.007485, 0.862000; Pr[9] << -0.271778, 0.226662, 0.536290;
Pk[10] << -0.126600, -0.040141, 0.889000; Pr[10] << -0.271813, 0.185335, 0.536856;
Pk[11] << -0.128432, 0.162474, 0.891000; Pr[11] << -0.271816, 0.379778, 0.422282;
Pk[12] << -0.020814, 0.214261, 0.705000; Pr[12] << -0.161190, 0.509979, 0.548749;

Pk[13] << -0.049375, 0.193703, 0.729000; Pr[13] << -0.189596, 0.478652, 0.541590;



//Pk[10] << -0.149770, -0.085097, 0.980000; Pr[10] << -0.211944, 0.177851, 0.552139;

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
    cloud_in->points[i].x = Pk[i](0);
    cloud_in->points[i].y = Pk[i](1);
    cloud_in->points[i].z = Pk[i](2);

    cloud_out->points[i].x = Pr[i](0);
    cloud_out->points[i].y = Pr[i](1);
    cloud_out->points[i].z = Pr[i](2);

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
    pt = ( (Tf.block(0,0,3,3))*Pk[i] + Tf.block(0,3,3,1) );
    std::cout << "[DEBUG] Orig point: "<< Pr[i].transpose() <<
      " , Tf: "<< pt.transpose() <<" error: "<< (Pr[i] - pt).norm() << std::endl;

  }




 return (0);
}

