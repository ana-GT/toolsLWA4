#include <vector>
#include<iostream>
#include <Eigen/Geometry>

void svdApproach( std::vector<Eigen::Vector3d> Pk,
		  std::vector<Eigen::Vector3d> Pw,
		  Eigen::Isometry3d &Tf );

double getErr( std::vector<Eigen::Vector3d> _Pk,
	       std::vector<Eigen::Vector3d> _Pr,
	       Eigen::Isometry3d Tf,
	       bool verbose = false );

int main( int argc, char* argv[] ) {

  int count = 0;
  int n = 19;
  
  std::vector<Eigen::Vector3d> Pk(n);
  std::vector<Eigen::Vector3d> Pr(n);
  

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

  std::vector<Eigen::Vector3d> Pks(4);
  std::vector<Eigen::Vector3d> Prs(4);
  std::vector<double> err;
  Eigen::Isometry3d Tf;
  double minErr = 1000;
  double tempErr;
  Eigen::Isometry3d minTf;

  for( int i = 0; i < n; ++i  ) {
    for(int j = i+1; j < n; ++j ) {
      for(int k = j+1; k < n; ++k ) {
	for(int m = k+1; m< n; ++m ) {
	  Pks[0] = Pk[i]; Prs[0] = Pr[i];
	  Pks[1] = Pk[j]; Prs[1] = Pr[j];
	  Pks[2] = Pk[k]; Prs[2] = Pr[k];
	  Pks[3] = Pk[m]; Prs[3] = Pr[m];
	  count++;
	  svdApproach( Pks, Prs, Tf );
	  tempErr = getErr(Pk, Pr, Tf);
	  err.push_back( tempErr  );
	  if( tempErr < minErr ) { minErr = tempErr; minTf = Tf; }

	  if( Tf.translation()(2) > 1.3 && Tf.linear()(0,0) < 0 && Tf.linear()(1,1) > 0 ) { 
	   // std::cout << "Seems good option: "<<std::endl;
	  //  std::cout << Tf.matrix() << std::endl;
	  //  std::cout << "with error: " << getErr(Pk, Pr, Tf, true) << std::endl;
	  }

	}	
      }      
    }
  }

  std::cout << "Min error: "<< minErr << " with tf: \n"<<std::endl;
  std::cout << minTf.matrix() << std::endl;
  

  std::cout << "Num counts: "<< count << std::endl;

}

double getErr( std::vector<Eigen::Vector3d> _Pk,
	       std::vector<Eigen::Vector3d> _Pr,
	       Eigen::Isometry3d Tf, bool verbose ) {
  
  double err = 0;
  double eachErr;
  for( int i = 0; i < _Pk.size(); ++i ) {
    Eigen::Vector3d pt;
    pt = ( Tf.linear()*_Pk[i] + Tf.translation() );
    eachErr = (_Pr[i] - pt).norm(); 
    err +=  eachErr;    
    if( verbose ) {
      std::cout << "["<<i<<"] Pk: "<< _Pk[i].transpose() << " Pr: "<< _Pr[i].transpose() << " Prn: "<< pt.transpose() <<" err: "<< eachErr << std::endl;
    }
  }



  return err / (double)_Pk.size();
}

/**
 * @function svdApproach
 */
void svdApproach( std::vector<Eigen::Vector3d> Pk,
		  std::vector<Eigen::Vector3d> Pw,
		  Eigen::Isometry3d &Tf ) {

  int numMatches = Pk.size();

  // 1. Compute the weighted centroids of both kinect and world sets
  // (all points same weight in our case)
  Eigen::Vector3d xm; xm << 0, 0, 0; // Points from Kinect
  Eigen::Vector3d ym; ym << 0, 0, 0; // Points from kinematics

  for( int i = 0; i < numMatches; ++i ) {    
    xm += Pk[i];
    ym += Pw[i];
  }
  xm = xm / numMatches; ym = ym / numMatches;

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
  Eigen::Matrix3d Rot;
  Rot = V*M*(U.transpose());

  Eigen::Vector3d trans;
  trans = ym - Rot*xm;

  Tf.setIdentity();
  Tf.linear() = Rot;
  Tf.translation() = trans;
  
}

