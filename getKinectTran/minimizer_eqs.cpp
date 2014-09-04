/**
 * @file minimizer_eqs.cpp
 */
#include <Eigen/Core>
#include <vector>
#include <levmar/levmar.h>


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
void minimize();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  int n = 11;
  std::vector<Eigen::Vector3d> Pk(n);
  std::vector<Eigen::Vector3d> Pr(n);

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
  
  minimize( n, Pk, Pr );

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
  p[0] = 0; p[1] = 3.14; p[2] = 0;
  p[3] = 0.1; p[4] = 0.65; p[z] = 1.4;

  // Set limits
  double ub[m]; double lb[m];
  for( int i = 0; i < 3; ++i ) { lb[i] = -M_PI; ub[i] = M_PI; }
  lb[3] = -2.0; ub[3] = 2.0; // tx
  lb[4] = 0.0; ub[4] = 2.0; // ty
  lb[5] = 0.8; ub[5] = 2.0; // tz
  
  int ret;
  ret = dlevmar_bc_der( levmar_fx, levmar_jac, 
			p, y, m, n, 
			lb, ub,
			NULL, 1000, opts, info,
			NULL, NULL, (void*)&data );

}


void levmar_fx( double *p, double* x, int m, int n, void *data ) {

  double xr, yr, zr, xk, yk, zk;
  double t2, t3, t4, t5, t6, t7, t9, t10;

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
  double t31, t32, t33, t34, t35, t36, t37, t38;

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
