/**
 * @file pir_lisp.h
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdint.h>
#include <ach.h>
#include <piranha.h>

enum {
    LEFT = 0,
    RIGHT,
    LR
};

/** traj_point_t */
struct traj_point_t{

    Eigen::VectorXd q;
    double dt;
};

/******************* HELPERS ********************************/
traj_point_t make_trajq_point( const Eigen::VectorXd &_q,
			       const double &_dt );


/**
 * @class pir_lisp
 */
class pir_lisp {

 public:
    pir_lisp();
    ~pir_lisp();
    bool pir_start();
    bool pir_stop();

    bool pir_tuck( int _side = LEFT );

    bool pir_trajq_side( int _side, 
			 std::vector<traj_point_t> _traj );

    bool pir_trajq( int _side,
		    std::vector<traj_point_t> _traj );

    traj_point_t pir_trajq_side_point( int _side,
				       traj_point_t _p );
 private:

    ach_channel_t ctrl_channel;
    ach_channel_t state_channel;
    ach_channel_t config_channel;
    ach_channel_t complete_channel;
    

 public:
    Eigen::Quaterniond R_DOWN;
    Eigen::Quaterniond R_UP_IN;
    Eigen::Quaterniond R_LEFT;
    Eigen::Quaterniond R_LEFT_IN;
    Eigen::Quaterniond R_RIGHT;
    Eigen::Quaterniond R_RIGHT_IN;
    Eigen::Quaterniond WRIST_E_SDH;
    
    
    Eigen::VectorXd Q_STORE_L;
    Eigen::VectorXd Q_UP_L;
    Eigen::VectorXd Q_OVER_L;
    Eigen::VectorXd Q_GO_L;
    Eigen::VectorXd Q_GO_R;
    

};


////////// HELPERS ///////////////
traj_point_t make_trajq_point( const Eigen::VectorXd &_q,
			       const double &_dt );
