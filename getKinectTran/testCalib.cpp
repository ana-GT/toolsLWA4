

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include <stdio.h>

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pk;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pr;


int main( int argc, char* argv[] ) {

  // Fill Pk
  Pk.resize(19); 
  Pr.resize(19);

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



  int numMatches = Pk.size();

  if( numMatches < 4 ) {
    printf("\t * [ERROR] You need at least 4 points to get the Twk!\n");
    return 1;
  }

  // 1. Compute the weighted centroids of both kinect and world sets
  // (all points same weight in our case)
  Eigen::Vector3d xm; xm << 0, 0, 0; // Points from Kinect
  Eigen::Vector3d ym; ym << 0, 0, 0; // Points from kinematics

  for( int i = 0; i < numMatches; ++i ) {    
    xm += Pk[i];
    ym += Pr[i];
  }

  std::cout << "Bef, xm: "<< xm.transpose() << " ym: "<< ym.transpose() << std::endl;
  xm = xm / numMatches; ym = ym / numMatches;
  std::cout << "xm: "<< xm.transpose() << " ym: "<< ym.transpose() << std::endl;

  // 2. Compute centered vectors
  Eigen::MatrixXd X(3,numMatches);
  Eigen::MatrixXd Y(3,numMatches);	

  for( int i = 0; i < numMatches; ++i ) {
    X.col(i) = Pk[i] - xm;
    Y.col(i) = Pr[i] - ym;
  }

  // 3. Compute the 3x3 covariance matrix
  Eigen::Matrix3d S;
  S = X*(Y.transpose()); // X*W*Yt -> W is identity


  // 4. Compute the singular value decomposition
  Eigen::JacobiSVD<Eigen::MatrixXd> svd( S, Eigen::ComputeThinU | Eigen::ComputeThinV );
  Eigen::Matrix3d U; Eigen::Matrix3d V;
  U = svd.matrixU(); V = svd.matrixV();
  
  Eigen::Matrix3d temp; temp = V*(U.transpose()); 
  Eigen::Matrix3d M; M.setIdentity(); M(2,2) = temp.determinant();
  std::cout << "This should be 1: "<< M(2,2) << std::endl;
  Eigen::Matrix3d Rot;
  Rot = V*M*(U.transpose());

  Eigen::Vector3d trans;
  trans = ym - Rot*xm;

  std::cout << "\t * Rotation calculated: \n"<< Rot << std::endl;
  std::cout << "\t * Translation calculated: \n"<< trans << std::endl;

  for( int i = 0; i < numMatches; ++i ) {
    
    std::cout << "[DEBUG] Orig point: "<< Pr[i].transpose() <<
      " and with Tf: "<< ( Rot*Pk[i] + trans ).transpose() << std::endl;

  }


}
