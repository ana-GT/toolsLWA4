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
  int n = 11;
  
  std::vector<Eigen::Vector3d> Pk(n);
  std::vector<Eigen::Vector3d> Pr(n);
  
  Pk[0] << 0.176150, 0.313715, 0.966000; Pr[0] << 0.050160, 0.468393, 0.288904;
  Pk[1] << 0.170304, 0.110383, 0.908000; Pr[1] << 0.045830, 0.310416, 0.446611;
  Pk[2] << 0.174337, -0.028176, 1.014000; Pr[2] << 0.054553, 0.126199, 0.432561;
  Pk[3] << 0.164567, -0.046508, 1.030000; Pr[3] << 0.044347, 0.105062, 0.433419;
  Pk[4] << -0.128743, -0.007485, 0.862000; Pr[4] << -0.271778, 0.226662, 0.536290; 
  Pk[5] << 0.007563, 0.127061, 0.871000; Pr[5] << -0.130705, 0.334754, 0.468285;
  Pk[6] << -0.126600, -0.040141, 0.889000; Pr[6] << -0.271813, 0.185335, 0.536856;
  Pk[7] << -0.128432, 0.162474, 0.891000; Pr[7] << -0.271816, 0.379778, 0.422282;  
  Pk[8] << -0.127372, -0.150694, 1.033000; Pr[8] << -0.272122, -0.000054, 0.472733;
  Pk[9] << 0.168325, 0.173799, 0.788000; Pr[9] << 0.024200, 0.415512, 0.513829;
  
  Pk[10] << 0.121617, 0.081078, 0.753000; Pr[10] << -0.015849, 0.348099, 0.594581;
  /*
  Pk[5] << 0.236176, 0.043827, 0.701000; Pr[5] << 0.100861, 0.341373, 0.659949;

  Pk[8] << -0.020814, 0.214261, 0.705000; Pr[8] << -0.161190, 0.509979, 0.548749;
  Pk[12] << -0.049375, 0.193703, 0.729000; Pr[12] << -0.189596, 0.478652, 0.541590;
  */

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
	    std::cout << "Seems good option: "<<std::endl;
	    std::cout << Tf.matrix() << std::endl;
	    std::cout << "with error: " << getErr(Pk, Pr, Tf, true) << std::endl;
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

