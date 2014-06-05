/**
 * @file piranha_filter.cpp
 * @brief Combines all info from arm/hand/FT in a big combo channel pir_state
 */
#include "piranha_filter.h"
#include <sns.h>

#include <time.h>

/**
 * @function
 * @brief
 */
piranha_filter::piranha_filter() {

}

/**
 * @function
 * @brief
 */
piranha_filter::~piranha_filter() {

}

/**
 * @function
 * @brief
 */
bool piranha_filter::init() {

    // Set memory space for the filter_data_t struct
    memset( &mFd, 0, sizeof(mFd) );

    // Read args to set verbosity of SNS
    /*
    for( int c; -1 != (c = getopt(argc, argv, "V?hH" SNS_OPTSTRING)); ) {
        switch(c) {
            SNS_OPTCASES;
        default:
            SNS_DIE( "Invalid argument: %s\n", optarg );
        }
    }    
    */

    // Init and start sns
    /*    sns_init();
    sns_start(); */
    
    // Open channels to query
    sns_chan_open( &mFd.chan_state_torso, "state-torso", NULL );
    sns_chan_open( &mFd.chan_state_left, "state-left", NULL );
    sns_chan_open( &mFd.chan_state_right, "state-right", NULL );
    sns_chan_open( &mFd.chan_state_left, "state-left", NULL );

    sns_chan_open( &mFd.chan_sdhstate_left, "sdhstate-left", NULL );
    sns_chan_open( &mFd.chan_sdhstate_right, "sdhstate-right", NULL );
 
    sns_chan_open( &mFd.chan_ft_left, "ft-left", NULL );  
    sns_chan_open( &mFd.chan_ft_right, "ft-right", NULL );   

    sns_chan_open( &mFd.chan_ftbias[PIR_LEFT], "ft-bias-left", NULL );  
    sns_chan_open( &mFd.chan_ftbias[PIR_RIGHT], "ft-bias-right", NULL );

    sns_chan_open( &mFd.chan_state_pir, "pir-state", NULL );    
    sns_chan_open( &mFd.chan_config, "pir-config", NULL );    

    /*
    
    {
        ach_channel_t *chans[] = {&cx.chan_state_left, &cx.chan_state_torso, NULL};
        sns_sigcancel( chans, sns_sig_term_default );
    }
    
    // Init constants 
    {
        // F/T rotation
        double R0[9] = { 0, 1, 0,
                         0, 0, 1,
                         1, 0, 0 };
        assert(aa_tf_isrotmat(R0));
        double r0[4];
        aa_tf_rotmat2quat(R0, r0);

        //double Rrot[9];
        //aa_tf_zangle2rotmat(15*M_PI/180, Rrot);
        //aa_tf_9mul( R0, Rrot, cx.R_ft_rel );

        double r_rel[4];
        aa_tf_zangle2quat(LWA4_FT_ANGLE, r_rel);
        aa_tf_qmul( r0, r_rel, cx.r_ft_rel );
    }


    // Register signal handler 
    {
        struct sigaction act;
        memset(&act, 0, sizeof(act));
        act.sa_handler = &sighandler_hup;
        if( sigaction(SIGHUP, &act, NULL) ) {
            SNS_DIE( "Could not install signal handler\n");
        }
    }
    */
    
}

/**
 * @function run
 * @brief Run a loop reading the info from arms/hands/FT and outputting to pir
 * channel with all the data together
 */
void piranha_filter::run() {

    while( !sns_cx.shutdown ) {

	// Update readings
	this->update();
	// If F/T must be rebiased, do so
	if( mFd.rebias ) {
	    bias_ft();
	}

	// Memory stuff [TO-READ]
	aa_mem_region_local_release();
    }

}

/**
 * @function update
 * @brief Reads info from hands/arms/FT and outputs to pir channel
 */
void piranha_filter::update(void) {

    int updated_flag = 0;

    //-- If cannot get updated time, log error
    if( clock_gettime( ACH_DEFAULT_CLOCK,
		       &mFd.now ) ) {
	SNS_LOG( LOG_ERR,
		 "clock_gettime failed: '%s' \n", strerror(errno) );		 
    }

    //-- Update time in SNS
    struct timespec timeout = sns_time_add_ns( mFd.now, 1000*1000*1);
    
    //-- Read information from arms / hands (AXES)
    int u_l = update_n( 7, PIR_AXIS_L0, &mFd.chan_state_left, &timeout );
    int u_r = update_n( 7, PIR_AXIS_R0, &mFd.chan_state_right, &timeout );
    updated_flag = updated_flag || u_l || u_r;

    int u_sl = update_n( 7, PIR_AXIS_SDH_L0, &mFd.chan_sdhstate_left, &timeout );
    int u_sr = update_n( 7, PIR_AXIS_SDH_R0, &mFd.chan_sdhstate_right, &timeout );
    updated_flag = updated_flag || u_sl || u_sr;

    //-- Read information from torso ( Dude, are we using you?)
    int u_t = update_n( 1, PIR_AXIS_T, &mFd.chan_state_torso, &timeout );
    updated_flag = updated_flag || u_t;

    //-- Read information from F/T
    int u_fl = update_ft( mFd.F_raw[PIR_LEFT], &mFd.chan_ft_left, &timeout );
    int u_fr = update_ft( mFd.F_raw[PIR_RIGHT], &mFd.chan_ft_right, &timeout );

    updated_flag = updated_flag || u_fl || u_fr;

}

/**
 * @function update_n
 * @brief Update mFd with _n motors info from *chan
 */
int piranha_filter::update_n( size_t _n,
			      size_t _i,
			      ach_channel_t *_chan,
			      struct timespec *_ts ) {

}

/**
 * @function update_ft
 * @brief Update F/T info in mFd with channel _chan
 */
int piranha_filter::update_ft( double *_F,
			       ach_channel_t *_chan,
			       struct timespec *_ts ) {

}


/**
 * @function
 * @brief
 */
void piranha_filter::sighandler_hup( int _sig ) {

}

/**
 * @function
 * @brief
 */
int piranha_filter::bias_ft( void ) {

}

