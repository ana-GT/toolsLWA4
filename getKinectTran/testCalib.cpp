

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include <stdio.h>

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pk;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pw;


int main( int argc, char* argv[] ) {

  // Fill Pk
  Pk.resize(4); 
  Pk[0] << 0.213541, 0.212068, 0.848000;
  Pk[1] << -0.084807, 0.090220, 1.039000;
  Pk[2] << 0.008206, -0.137857, 0.945000;
  Pk[3] << -0.075120, -0.122756, 1.055000;

  Pw.resize(4);
  Pw[0] << -0.0250601, 0.533556, 0.611336;
  Pw[1] << -0.305515, 0.331292, 0.409072;
  Pw[2] << -0.235545, 0.13622, 0.606205;
  Pw[3] << -0.301561, 0.103797, 0.488168;

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
    ym += Pw[i];
  }

  std::cout << "Bef, xm: "<< xm.transpose() << " ym: "<< ym.transpose() << std::endl;
  xm = xm / numMatches; ym = ym / numMatches;
  std::cout << "xm: "<< xm.transpose() << " ym: "<< ym.transpose() << std::endl;

  // 2. Compute centered vectors
  Eigen::MatrixXd X(3,numMatches);
  Eigen::MatrixXd Y(3,numMatches);	

  for( int i = 0; i < numMatches; ++i ) {
    X.col(i) = Pk[i] - xm;
    Y.col(i) = Pw[i] - ym;
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
    
    std::cout << "[DEBUG] Orig point: "<< Pw[i].transpose() <<
      " and with Tf: "<< ( Rot*Pk[i] + trans ).transpose() << std::endl;

  }


}
