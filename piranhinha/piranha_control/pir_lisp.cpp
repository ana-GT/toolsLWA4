/**
 * @file pir_lisp.cpp
 */

#include "pir_lisp.h"
#include <iostream>

/**
 *
 */
pir_lisp::pir_lisp() {

    Q_STORE_L.resize(7);
    Q_STORE_L << 0.5*M_PI, -0.5*M_PI, 0, 0, 0, 0, 0;
    
    Q_UP_L.resize(7);
    Q_UP_L << 0.5*M_PI, -0.2*M_PI, 0, -0.2*M_PI, 0, 0.55*M_PI, 0;

    Q_OVER_L.resize(7);
    Q_OVER_L << 0.1-M_PI, -0.2*M_PI, 0, -0.2*M_PI, 0, 0.55*M_PI, 0;

    Q_GO_L.resize(7);
    Q_GO_L << 0.6128, -1.0637, -0.7396, -1.2927, -0.9403, 1.3764, 0.5271;
    
    Q_GO_R.resize(7);
    Q_GO_R << -0.5366, 0.9483, 0.5890, 1.2863, 1.0864, -1.4666, -1.5854;
}

pir_lisp::~pir_lisp() {

}


/**
 * @function pir_start
 * @brief Open pir channels
 */
bool pir_lisp::pir_start() {
        
    int r;

    r = ach_open( &ctrl_channel, "pir-ctrl", NULL );
    if( r != ACH_OK ) { 
	std::cout << "\t [ERROR] Opening pir-ctrl" <<std::endl; 
	return false; 
    }

    r = ach_open( &config_channel, "pir-config", NULL );
    if( r != ACH_OK ) { 
	std::cout << "\t [ERROR] Opening pir-config" <<std::endl; 
	return false; 
    }

    r = ach_open( &config_channel, "pir-state", NULL );
    if( r != ACH_OK ) { 
	std::cout << "\t [ERROR] Opening pir-state" <<std::endl; 
	return false; 
    }

    r = ach_open( &config_channel, "pir-complete", NULL );
    if( r != ACH_OK ) { 
	std::cout << "\t [ERROR] Opening pir-complete" <<std::endl; 
	return false; 
    }
   

    return true;
}

/**
 * @function pir_stop
 * @brief Close control channel
 */
bool pir_lisp::pir_stop() {
    ach_close( &ctrl_channel );
}

/**
 * @function pir_tuck
 * @brief Set the arm back to home position
 */
bool pir_lisp::pir_tuck( int _side ) {

    std::vector<traj_point_t> traj;
    traj.push_back( make_trajq_point(Q_GO_L, 5.0) );
    traj.push_back( make_trajq_point(Q_OVER_L, 3.0) );
    traj.push_back( make_trajq_point(Q_UP_L, 5.0) );

    pir_trajq_side( _side, traj );
    

}

/**
 * @function pir_trajq_side
 */
bool pir_lisp::pir_trajq_side( int _side, 
			       std::vector<traj_point_t> _traj ) {

    std::vector<traj_point_t> points;
    for( int i = 0; i < _traj.size(); ++i ) {
	points.push_back( pir_trajq_side_point(_side, _traj[i]) );
    }
    
    return pir_trajq( _side, points );		      
}

bool pir_lisp::pir_trajq( int _side,
			  std::vector<traj_point_t> _traj ) {
    /*    pir_message( side_case( _side, "trajq"),
	  trajq-point-data(_traj) ); */
}


/**
 * @function get_pir_trajq_side_points 
 */
traj_point_t pir_lisp::pir_trajq_side_point( int _side,
					     traj_point_t _p ) {
    traj_point_t point;
    
    switch( _side ) {
    case LEFT: {
	point = _p; break;
    }
    case RIGHT: {
	point = make_trajq_point( -1*_p.q,_p.dt); 
	break;
    }
    case LR: {
	Eigen::VectorXd pq( _p.q.size()*2 );
	pq.topRows(_p.q.size()) = _p.q;
	pq.bottomRows(_p.q.size()) = -1*_p.q;
	point = make_trajq_point( pq,_p.dt); break;
    }
    }
    return point;
}


/////////////////// HELPERS //////////////////////////

/**
 * @function make_trajq_point
 * @brief Make a trajectory point (joint conf and time duration)
 */
traj_point_t make_trajq_point( const Eigen::VectorXd &_q,
			       const double &_dt ) {
    traj_point_t p; p.q = _q; p.dt = _dt;
    return p;
}
