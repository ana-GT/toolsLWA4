

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include <stdio.h>

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pk;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pr;


int main( int argc, char* argv[] ) {

  // Fill Pk
  Pk.resize(18); 
  Pr.resize(18);

  Pk[0] << 0.067431, 0.261792, 1.142000; Pr[0] << 0.050173, 0.468391, 0.288906;
  Pk[1] << 0.071252, 0.069425, 1.052000; Pr[1] << 0.045855, 0.310442, 0.446609;
  Pk[2] << 0.074836, -0.076806, 1.134000; Pr[2] << 0.054557, 0.126207, 0.432608;
  Pk[3] << 0.067195, -0.096840, 1.138000; Pr[3] << 0.044363, 0.105079, 0.433427;
  Pk[4] << 0.344242, -0.129873, 0.901000; Pr[4] << 0.321746, 0.166194, 0.682148;
  Pk[5] << 0.455632, 0.073227, 0.937000; Pr[5] << 0.435891, 0.348102, 0.571312;
  Pk[6] << 0.031052, 0.065208, 0.894000; Pr[6] << -0.015878, 0.348099, 0.594568;
  Pk[7] << 0.147269, 0.038290, 0.848000; Pr[7] << 0.100844, 0.341366, 0.659949;
  Pk[8] << 0.070793, 0.151465, 0.948000; Pr[8] << 0.024189, 0.415506, 0.513831;
  Pk[9] << -0.084277, 0.089544, 1.011000; Pr[9] << -0.130683, 0.334781, 0.468276;
  Pk[10] << -0.222540, -0.212693, 1.134000; Pr[10] << -0.272125, -0.000055, 0.472721;
 Pk[11] << -0.218230, -0.250427, 1.030000; Pr[11] << -0.271773, -0.000057, 0.582231;
 Pk[12] << -0.216146, -0.044250, 0.980000; Pr[12] << -0.271780, 0.226659, 0.536270;
 Pk[13] << -0.215993, -0.080126, 1.003000; Pr[13] << -0.271813, 0.185333, 0.536859;
 Pk[14] << -0.220659, 0.121990, 1.033000; Pr[14] << -0.271810, 0.379796, 0.422292;
 Pk[15] << -0.107034, 0.199203, 0.856000; Pr[15] << -0.161207, 0.509967, 0.548746;
 Pk[16] << -0.134487, 0.175751, 0.880000; Pr[16] << -0.189573, 0.478660, 0.541611;
 Pk[17] << 0.259745, 0.247756, 0.767000; Pr[17] << 0.207489, 0.588670, 0.648450;


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
