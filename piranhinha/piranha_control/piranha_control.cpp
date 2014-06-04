/**
 * @file piranha_control.cpp
 * @author Copied literally from pirctrl.c 
 */

#include <sns.h>


#include "piranha_control.h"

/**
 * @function piranha_control
 * @brief Constructor
 */
piranha_control::piranha_control() {

}


/**
 * @function ~piranha_control
 * @brief Destructor
 */
piranha_control::~piranha_control() {

}

/**
 * @function init_setup
 */
bool piranha_control::init_setup() {

    // Set log / error / message variables
    sns_init();

    // Reserve space of piranha struct and set time d
    memset( &mCx, 0, sizeof( mCx ) );
    mCx.dt = 1.0 / 250;

    
    // Start
    //sns_start();

    // Open channels
    sns_chan_open( &mCx.chan_js, "joystick", NULL );
    sns_chan_open( &mCx.chan_ctrl, "pir-ctrl", NULL );
    sns_chan_open( &mCx.chan_ref_torso, "ref-torso", NULL );
    sns_chan_open( &mCx.chan_ref_left, "ref-left", NULL );
    sns_chan_open( &mCx.chan_ref_right, "ref-right", NULL );
    sns_chan_open( &mCx.chan_sdhref_left, "sdhref-left", NULL );
    sns_chan_open( &mCx.chan_sdhref_right, "sdhref-right", NULL );
    sns_chan_open( &mCx.chan_state_pir, "pir-state", NULL );
    sns_chan_open( &mCx.chan_config, "pir-config", NULL );
    sns_chan_open( &mCx.chan_complete, "pir-complete", NULL );

    
    {
	ach_channel_t *chans[] = {&mCx.chan_state_pir,
				  &mCx.chan_js, NULL};
	sns_sigcancel( chans, sns_sig_term_default );
    }

    // Alloc messages
    mCx.msg_ref = sns_msg_motor_ref_alloc( PIR_MAX_MSG_AXES );
    mCx.msg_ref->mode = SNS_MOTOR_MODE_VEL;

    // Memory
    aa_mem_region_init( &mCx.modereg, 64*1024 );

    // Setup reflex controller
    for( size_t i = 0; i < PIR_AXIS_CNT; ++i ) {
	mCx.q_min[i] = -2*M_PI;
	mCx.q_max[i] = M_PI;
    }

    // Left / right controller
    for( int side = 0; side < 2; ++side ) {
	int lwa, sdh;
	PIR_SIDE_INDICES( side, lwa, sdh );
	(void) sdh;
	mCx.G[side].n_q = 7;
	mCx.G[side].J = mCx.state.J_wp[side];
	mCx.G[side].act.q = &mCx.state.q[lwa];
	mCx.G[side].act.dq = &mCx.state.dq[lwa];
	mCx.G[side].act.S = mCx.state.S_wp[side];
	mCx.G[side].act.F = mCx.state.F[side];
	mCx.G[side].ref.q = &mCx.ref.q[lwa];
	mCx.G[side].ref.dq = &mCx.ref.dq[lwa];	
	mCx.G[side].q_min = &mCx.q_min[lwa];
	mCx.G[side].q_max = &mCx.q_max[lwa];
	mCx.G[side].ref.S = AA_NEW0_AR( double, 8 );
	mCx.G[side].ref.F = AA_NEW0_AR( double, 6 );
	mCx.G[side].ref.dx = AA_NEW0_AR( double, 6 );
	mCx.G[side].act.dx = AA_NEW0_AR( double, 6 );

	for( size_t i = 0; i < 3; ++i ) {
	    mCx.G[side].x_min[i] = -10;
	    mCx.G[side].x_max[i] = 10;
	}
	mCx.G[side].F_max = 20;
    }

    // LEFT_RIGHT
    //_Static_assert( PIR_AXIS_L0 + 7 == PIR_AXIS_R0, "Invalid axis ordering" );
    mCx.G_LR.n_q = 14;
    //mCx.G_R.J =  mCx.state.J_wp_R;
    mCx.G_LR.act.q =  &mCx.state.q[PIR_AXIS_L0];
    mCx.G_LR.act.dq = &mCx.state.dq[PIR_AXIS_L0];
    //mCx.G_R.act.S = mCx.state.S_wp_R;
    //mCx.G_R.act.F = mCx.state.F_R;
    mCx.G_LR.ref.q =  &mCx.ref.q[PIR_AXIS_L0];
    mCx.G_LR.ref.dq = &mCx.ref.dq[PIR_AXIS_L0];
    mCx.G_LR.q_min = &mCx.q_min[PIR_AXIS_L0];
    mCx.G_LR.q_max = &mCx.q_max[PIR_AXIS_L0];
    //mCx.G_R.ref.S = AA_NEW0_AR( double, 8 );
    //mCx.G_R.ref.F = AA_NEW0_AR( double, 6 );
    //mCx.G_R.ref.dx = AA_NEW0_AR( double, 6 );
    //mCx.G_R.act.dx = AA_NEW0_AR( double, 6 );
    //for( size_t i = 0; i < 3; i ++ ) {
        //mCx.G_R.x_min[i] = -10;
        //mCx.G_R.x_max[i] = 10;
    //}
    //mCx.G_R.F_max = 20;

    // torso
    mCx.G_T.n_q = 1;
    mCx.G_T.J =  NULL;
    mCx.G_T.act.q =  &mCx.state.q[PIR_AXIS_T];
    mCx.G_T.act.dq = &mCx.state.dq[PIR_AXIS_T];
    mCx.G_T.act.S = NULL;
    mCx.G_T.act.F = NULL;
    mCx.G_T.ref.q =  &mCx.ref.q[PIR_AXIS_T];
    mCx.G_T.ref.dq = &mCx.ref.dq[PIR_AXIS_T];
    mCx.G_T.q_min = &mCx.q_min[PIR_AXIS_T];
    mCx.G_T.q_max = &mCx.q_max[PIR_AXIS_T];
    mCx.G_T.ref.S = NULL;
    mCx.G_T.ref.F = NULL;
    mCx.G_T.ref.dx = NULL;
    mCx.G_T.act.dx = NULL;
    for( size_t i = 0; i < 3; i ++ ) {
        mCx.G_T.x_min[i] = -10;
        mCx.G_T.x_max[i] = 10;
    }
    mCx.G_T.F_max = 20;



    rfx_ctrl_ws_lin_k_init( &mCx.Kx, 7 );
    AA_MEM_SET( mCx.Kx.q, 0.1, 7 );
    mCx.Kx.q[1] *= 5; // lower limits
    mCx.Kx.q[3] *= 5; // lower limits
    mCx.Kx.q[5] *= 5; // lower limits
    mCx.Kx.q[6] *= 5; // this module is most sensitive to limits
    //AA_MEM_SET( mCx.Kx.f, .003, 3 );
    //AA_MEM_SET( mCx.Kx.f+3, .000, 3 );
    //AA_MEM_SET( mCx.Kx.f, -.000, 6 );
    AA_MEM_SET( mCx.Kx.p, 1.0, 3 );
    AA_MEM_SET( mCx.Kx.p+3, 1.0, 3 );
    /* AA_MEM_SET( mCx.K.p, 0.0, 3 ); */
    /* AA_MEM_SET( mCx.K.p+3, 0.0, 3 ); */
    mCx.Kx.dls = .005;
    mCx.Kx.s2min = .01;
    printf("dls s2min: %f\n", mCx.Kx.s2min);
    printf("dls k: %f\n", mCx.Kx.dls);

    // joint
    mCx.Kq.n_q = 7;
    mCx.Kq.p = AA_NEW_AR( double, 7 );
    AA_MEM_SET( mCx.Kq.p, .5, 7 );

    mCx.Kq_lr.n_q = 14;
    mCx.Kq_lr.p = AA_NEW_AR( double, 14 );
    AA_MEM_SET( mCx.Kq_lr.p, .5, 14 );

    mCx.Kq_T.n_q = 1;
    mCx.Kq_T.p = AA_NEW_AR( double, 1 );
    AA_MEM_SET( mCx.Kq_T.p, 0, 1 );

    if( clock_gettime( ACH_DEFAULT_CLOCK, &mCx.now ) )
        SNS_LOG( LOG_ERR, "clock_gettime failed: '%s'\n", strerror(errno) );

    return true;
}
