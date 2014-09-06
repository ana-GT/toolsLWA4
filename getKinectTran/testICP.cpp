#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/StdVector>

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pk;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pr;
int n = 19;

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  Pk.resize(n);
  Pr.resize(n);

Pk[0] << 0.000000, 0.000000, 0.000000; Pr[0] << 0.177145, 0.526442, 0.642802;
Pk[1] << 0.266960, 0.292173, 0.854000; Pr[1] << 0.157259, 0.501950, 0.655204;
Pk[2] << 0.244141, 0.263376, 0.852000; Pr[2] << 0.136790, 0.475623, 0.666753;
Pk[3] << 0.224551, 0.235030, 0.862000; Pr[3] << 0.116029, 0.447476, 0.677244;
Pk[4] << 0.201762, 0.204774, 0.867000; Pr[4] << 0.095042, 0.417547, 0.686527;
Pk[5] << 0.181864, 0.171166, 0.880000; Pr[5] << 0.074030, 0.385876, 0.694385;
Pk[6] << 0.160565, 0.143582, 0.889000; Pr[6] << 0.053096, 0.352567, 0.700589;
Pk[7] << 0.119439, 0.082811, 0.917000; Pr[7] << 0.012207, 0.281583, 0.707415;
Pk[8] << 0.101213, 0.048974, 0.940000; Pr[8] << -0.007477, 0.244239, 0.707675;
Pk[9] << 0.081523, 0.019965, 0.958000; Pr[9] << -0.026397, 0.205985, 0.705704;
Pk[10] << 0.064277, -0.010149, 0.974000; Pr[10] << -0.044492, 0.167032, 0.701346;
Pk[11] << 0.050666, -0.038436, 1.006000; Pr[11] << -0.061655, 0.127643, 0.694532;
Pk[12] << 0.033987, -0.062607, 1.030000; Pr[12] << -0.077722, 0.088089, 0.685249;
Pk[13] << 0.018426, -0.088445, 1.061000; Pr[13] << -0.092640, 0.048667, 0.673480;
Pk[14] << 0.007579, -0.111787, 1.091000; Pr[14] << -0.106344, 0.009653, 0.659251;
Pk[15] << -0.003901, -0.136519, 1.123000; Pr[15] << -0.118765, -0.028667, 0.642644;
Pk[16] << -0.016019, -0.152181, 1.153000; Pr[16] << -0.129842, -0.066041, 0.623770;
Pk[17] << -0.026934, -0.176107, 1.193000; Pr[17] << -0.139580, -0.102195, 0.602757;
Pk[18] << -0.034094, -0.189649, 1.227000; Pr[18] << -0.147966, -0.136907, 0.579771;



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

