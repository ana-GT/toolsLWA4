/**
 * @file readHandStates.cpp
 * @brief Read current hand states
 */
#include <sns.h>
#include <ach.h>

#define SDH_AXES 7

/**
 * @function main
 */
int main( int argc, char* argv ) {
    
    // Open log channel and set some stuff
    sns_init();
    
    // Shoots a message to parent (Achcop?)
    sns_start();

    // Open sdh state channel
    sns_chan_open( chan_sdhstate_left, "sdh-state-left", NULL );
    sns_chan_open( chan_sdhstate_right, "sdh-state-right", NULL );

    {
	ach_channel_t *chans[] = {chan_sdhstate_left, chan_sdhstate_right, NULL};
	sns_sigcancel( chans, sns_sig_term_default );
    }

    // Alloc messages
    //msg_ref = sns_msg_motor_ref_alloc( SDH_AXES );
    //msg_ref->mode = SNS_MOTOR_MODE_POSE;

    while( !sns_cx.shutdown ) {
	update();
    }

    sns_end();
    return 0;

}
