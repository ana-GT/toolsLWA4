/**
 * @file minimizer_eqs.cpp
 */
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <levmar/levmar.h>
#include <iostream>

/**
 * @struct Contains the matching robot-kinect points info
 */
struct levmar_data {
  double* xr;
  double* yr;
  double* zr;
  double* xk;
  double* yk;
  double* zk;
  int num;
};

void levmar_fx( double *p, double* x, int m, int n, void *data );
void levmar_jac( double* p, double* jac,
		 int m, int n, void* data );
void minimize(int n,
	       std::vector<Eigen::Vector3d> Pk,
	       std::vector<Eigen::Vector3d> Pr);

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

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


  minimize( 8, Pk, Pr );

  return 0;
}

void minimize( int n,
	       std::vector<Eigen::Vector3d> Pk,
	       std::vector<Eigen::Vector3d> Pr ) {
  
  int m = 6; // Parameters of transformation [R,t]
  double p[m]; // Parameter values
  double y[n]; // Values we want to achieve

  double opts[LM_OPTS_SZ];
  double info[LM_INFO_SZ];
  
  opts[0] = LM_INIT_MU;
  opts[1] = 1E-15;
  opts[2] = 1E-15;
  opts[3] = 1E-20;
  opts[4] = LM_DIFF_DELTA;

  struct levmar_data data;
  data.xr = new double[n];
  data.yr = new double[n];
  data.zr = new double[n];
  data.xk = new double[n];
  data.yk = new double[n];
  data.zk = new double[n];
  data.num = n;

  for( int i = 0; i < n; ++i ) {
    data.xr[i] = Pr[i](0);
    data.yr[i] = Pr[i](1);
    data.zr[i] = Pr[i](2);

    data.xk[i] = Pk[i](0);
    data.yk[i] = Pk[i](1);
    data.zk[i] = Pk[i](2);
  }

  // Set value you want to achieve. We want zero
  for( int i = 0; i < n; ++i ) {
    y[i] = 0;
  }

  // Initialize values for parameters p
  p[0] = 0.0; p[1] = 0.0; p[2] = 0.0;
  p[3] = 0.0; p[4] = 0.5; p[5] = 1.0;

  // Set limits
  
  double ub[m]; double lb[m];
  lb[0] = -3.14; ub[0] = 3.14;
  lb[1] = -3.14; ub[1] = 3.14;
  lb[2] = -3.14; ub[2] = 3.14;
  lb[3] = -2.0; ub[3] = 2.0; // tx
  lb[4] = 0.0; ub[4] = 2.0; // ty
  lb[5] = -2.0; ub[5] = 2.0; // tz
  
  int ret;
  ret = dlevmar_bc_der( levmar_fx, levmar_jac, 
			p, y, m, n, 
			lb, ub,
			NULL, 5000, opts, info, // opts
			NULL, NULL, (void*)&data );

  std::cout << "Levenberg returned in "<< info[5]<<" iterations. Reason: "<< info[6]<< " sumsq: "<< info[1] << "["<<info[0]<<"]"<< std::endl;
  std::cout << "Pars: "<< p[0]<< ", "<<p[1] <<", "<< p[2] << std::endl;

  Eigen::Isometry3d Tf;
  Tf = Eigen::Isometry3d::Identity();
  Tf.translation() << p[3], p[4], p[5];
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd( p[2], Eigen::Vector3d(0,0,1) )*Eigen::AngleAxisd( p[1], Eigen::Vector3d(0,1,0) )*Eigen::AngleAxisd( p[0], Eigen::Vector3d(1,0,0) );
  Eigen::Vector3d temp; temp = rot.col(0);
  //rot.col(0) = rot.col(1);
  //rot.col(1) = temp;
  Tf.linear() = rot;

  std::cout << "Transformation of Kinect w.r.t. world: \n"<< Tf.matrix() << std::endl;

    for( int i = 0; i < Pk.size(); ++i ) {
    Eigen::Vector3d pt;
    pt = ( Tf.linear()*Pk[i] + Tf.translation() );
    std::cout << "[DEBUG] Pr["<<i<<"]: "<< Pr[i].transpose() <<
      " , Tf(Pk["<<i<<"]): "<< pt.transpose() <<" error: "<< (Pr[i] - pt).norm() << std::endl;

  }
}


void levmar_fx( double *p, double* x, int m, int n, void *data ) {

  double xr, yr, zr, xk, yk, zk;
  double t2, t3, t4, t5, t6, t7, t8, t9, t10;

  struct levmar_data *dptr;
  dptr = (struct levmar_data*) data;

  for( int i = 0; i < n; ++i ) {
    
    xr = dptr->xr[i];
    yr = dptr->yr[i];
    zr = dptr->zr[i];
    
    xk = dptr->xk[i];
    yk = dptr->yk[i];
    zk = dptr->zk[i];
    
    // ra = p[0] pa = p[1] ya = p[2]
    // tx = p[3] ty = p[4] pz = p[5]
    t2 = sin(p[0]);
    t3 = sin(p[2]);
    t4 = cos(p[0]);
    t5 = cos(p[2]);
    t6 = sin(p[1]);
    t8 = cos(p[1]);
    t7 = p[3]-xr-yk*(t3*t4-t2*t5*t6)+zk*(t2*t3+t4*t5*t6)+t5*t8*xk;
    t9 = p[4]-yr+yk*(t4*t5+t2*t3*t6)-zk*(t2*t5-t3*t4*t6)+t3*t8*xk;
    t10 = p[5]-zr-t6*xk+t2*t8*yk+t4*t8*zk;
    
    x[i] = t7*t7+t9*t9+t10*t10;
  }
}

void levmar_jac( double* p, double* jac,
		 int m, int n, void* data ) {

  double xr, yr, zr, xk, yk, zk;
  double t12, t13, t14, t15, t16, t17, t18, t19, t20;
  double t21, t22, t23, t24, t25, t26, t27, t28, t29, t30;
  double t31, t32, t33, t34, t35, t36, t37, t38, t39, t40;

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;


  // ra = p[0] pa = p[1] ya = p[2]
  // tx = p[3] ty = p[4] pz = p[5] 
  for( int i = 0; i < n; ++i ) {

    xr = dptr->xr[i];
    yr = dptr->yr[i];
    zr = dptr->zr[i];
    
    xk = dptr->xk[i];
    yk = dptr->yk[i];
    zk = dptr->zk[i];


    t12 = cos(p[1]);
    t13 = cos(p[0]);
    t14 = sin(p[0]);
    t15 = sin(p[1]);
    t16 = sin(p[2]);
    t17 = cos(p[2]);
    t18 = t13*t16;
    t28 = t14*t15*t17;
    t19 = t18-t28;
    t20 = t14*t16;
    t21 = t13*t15*t17;
    t22 = t20+t21;
    t23 = t13*t17;
    t24 = t14*t15*t16;
    t25 = t23+t24;
    t26 = t14*t17;
    t36 = t13*t15*t16;
    t27 = t26-t36;
    t29 = t22*zk;
    t30 = t12*t17*xk;
    t40 = t19*yk;
    t31 = t29+t30-t40+p[3]-xr;
    t32 = t12*t13*zk;
    t33 = t12*t14*yk;
    t34 = t32+t33+p[5]-zr-t15*xk;
    t35 = t25*yk;
    t37 = t12*t16*xk;
    t39 = t27*zk;
    t38 = t35+t37-t39+p[4]-yr;
    
    jac[6*i+0] = t34*(t12*t13*yk-t12*t14*zk)*2.0+t31*(t22*yk+t19*zk)*2.0-t38*(t27*yk+t25*zk)*2.0;
    jac[6*i+1] = t34*(t12*xk+t14*t15*yk+t13*t15*zk)*-2.0+t31*(-t15*t17*xk+t12*t14*t17*yk+t12*t13*t17*zk)*2.0+t38*(-t15*t16*xk+t12*t14*t16*yk+t12*t13*t16*zk)*2.0;
    jac[6*i+2] = t38*(t29+t30-t40)*2.0-t31*(t35+t37-t39)*2.0;
    jac[6*i+3] = p[3]*2.0-xr*2.0-t19*yk*2.0+t22*zk*2.0+t12*t17*xk*2.0;
    jac[6*i+4] = p[4]*2.0-yr*2.0+t25*yk*2.0-t27*zk*2.0+t12*t16*xk*2.0;
    jac[6*i+5] = p[5]*2.0-zr*2.0-t15*xk*2.0+t12*t14*yk*2.0+t12*t13*zk*2.0;
  } // end for

}
