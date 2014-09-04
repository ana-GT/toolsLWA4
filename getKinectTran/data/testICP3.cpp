#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/StdVector>

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pk;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pr;
int n = 11;

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  Pk.resize(n);
  Pr.resize(n);

Pk[0] << 0.071398, 0.259809, 1.142000; Pr[0] << 0.050184, 0.468419, 0.288886;

Pk[1] << 0.070981, 0.070981, 1.048000; Pr[1] << 0.045854, 0.310434, 0.446621;

Pk[2] << 0.072867, -0.078775, 1.134000; Pr[2] << 0.054560, 0.126212, 0.432608;

Pk[3] << 0.063465, -0.097180, 1.142000; Pr[3] << 0.044365, 0.105079, 0.433429;

//Pk[4] << 0.347342, -0.132021, 0.905000; Pr[4] << 0.321748, 0.166193, 0.682149;

//Pk[5] << 0.455632, 0.074854, 0.937000; Pr[5] << 0.435891, 0.348101, 0.571315;

Pk[4] << 0.032495, 0.064990, 0.891000; Pr[4] << -0.015862, 0.348098, 0.594576;

Pk[5] << 0.147269, 0.038290, 0.848000; Pr[5] << 0.100851, 0.341364, 0.659953;

Pk[6] << 0.069293, 0.151785, 0.950000; Pr[6] << 0.024191, 0.415510, 0.513822;
 
Pk[7] << -0.086033, 0.091300, 1.011000; Pr[7] << -0.130680, 0.334779, 0.468280;

Pk[8] << -0.214444, -0.042548, 0.980000; Pr[8] << -0.271782, 0.226661, 0.536273;

Pk[9] << -0.222453, 0.121990, 1.033000; Pr[9] << -0.271820, 0.379783, 0.422279;
 
Pk[10] << -0.107034, 0.199203, 0.856000; Pr[10] << -0.161211, 0.509962, 0.548747;




  // Fill in the CloudIn data
  cloud_in->width    = n-4;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);

  cloud_out->width    = n-4;
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
    std::cout << "[DEBUG] Orig robot point: "<< Pr[i].transpose() <<
      " , Tf from kinect: "<< pt.transpose() <<" error: "<< (Pr[i] - pt).norm() << std::endl;

  }

  printf("TESTTTTTTTTTTTTTTTTTTTTTTTT\n");
  Tf <<  0.975264, 0.0787744, 0.206533, -0.390849,
    0.00822639, 0.920764, -0.390034, 0.650055,
    0.220893, -0.382086, -0.897341, 1.41051,
    0, 0, 0, 1;

  for( int i = 0; i < n; ++i ) {
    Eigen::Vector3d pt;
    pt = ( (Tf.block(0,0,3,3))*Pk[i] + Tf.block(0,3,3,1) );
    std::cout << "[PREV] Orig robot point: "<< Pr[i].transpose() <<
      " , Tf from kinect: "<< pt.transpose() <<" error: "<< (Pr[i] - pt).norm() << std::endl;

  }
  



 return (0);
}

