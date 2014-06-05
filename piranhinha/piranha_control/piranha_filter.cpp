/**
 * @file piranha_filter.cpp
 * @brief Combines all info from arm/hand/FT in a big combo channel pir_state
 */
#include "piranha_filter.h"
#include <sns.h>

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
    
    // Init and start sns
    /*    sns_init();
    sns_start(); */
    
    // Open channels to query
    sns_chan_open( &mFd.chan_state_torso, "state-torso", NULL );
    sns_chan_open( &mFd.chan_state_left, "state-left", NULL );
    sns_chan_open( &mFd.chan_state_right, "state-right", NULL );
    sns_chan_open( &mFd.chan_state_left, "state-left", NULL );

    sns_chan_open( &mFd.chan_sdhstate_left, "sdhstate-left", NULL );

}

/**
 * @function
 * @brief
 */
void piranha_filter::update(void) {
}

/**
 * @function
 * @brief
 */
int piranha_filter::update_n( size_t _n,
			      size_t _i,
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

