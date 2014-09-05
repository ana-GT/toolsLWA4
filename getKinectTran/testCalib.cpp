

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include <stdio.h>

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pk;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pr;


int main( int argc, char* argv[] ) {

  // Fill Pk
  Pk.resize(14); 
  Pr.resize(14);

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
