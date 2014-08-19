/**
 * @file readHandStates.cpp
 * @brief Read current hand states
 */
#include <unistd.h>
#include <sns.h>
#include <ach.h>

#define SDH_AXES 7

enum SDH_SIDE {
    SDH_LEFT= 0,
    SDH_RIGHT = 1
};

ach_channel_t chan_sdhstate_left;
ach_channel_t chan_sdhstate_right;
ach_channel_t chan_sdhref_left;
ach_channel_t chan_sdhref_right;
double ql[SDH_AXES]; double dql[SDH_AXES];
double qr[SDH_AXES]; double dqr[SDH_AXES];
struct timespec now;

//***********************************************
// Functions prototype
static void update( void );
static int update_n( size_t n,
		     double* q, double *dq,
		     ach_channel_t *chan,
		     struct timespec *ts );
void sdh_pos( double* x,
	      int side = SDH_LEFT,
	      double tsec = 5 );

/************************************************
 * @function main
 *************************************************/
int main( int argc, char* argv[] ) {
    
    // Open log channel and set some stuff
    sns_init();
    
    // Shoots a message to parent (Achcop?)
    sns_start();

    // Open sdh state channels
    sns_chan_open( &chan_sdhstate_left, "sdhstate-left", NULL );
    sns_chan_open( &chan_sdhstate_right, "sdhstate-right", NULL );
    sns_chan_open( &chan_sdhref_left, "sdhref-left", NULL );
    sns_chan_open( &chan_sdhref_right, "sdhref-right", NULL );


    {
	ach_channel_t *chans[] = { &chan_sdhstate_left, 
				   &chan_sdhstate_right, 
				   &chan_sdhref_left,
				   &chan_sdhref_right,
				   NULL};
	sns_sigcancel( chans, sns_sig_term_default );
    }

    // Control, move a finger
    double x[SDH_AXES];
    double tsec = 5.0;

    // Read 
    printf("Update 1 \n");
    update();
    aa_mem_region_local_release();
    sleep(1);
    printf("Update 2 \n");
    update();
    aa_mem_region_local_release();
    sleep(3);
    printf("Update 3 \n");
    update();
    aa_mem_region_local_release();
    sleep(3);


    // Make sure read left hand
    if( ql[0] > 1.5 && ql[2] > 0.7 && ql[4] < -1.4) {
	ql[4] = -1.0;

	std::cout << "Sending to pos: "<<std::endl;
	for( int i = 0; i < SDH_AXES; ++i ) {
	    std::cout << ql[i]<<" ";
	} std::cout << std::endl;

	printf("[LOOK] Sleep for 10 seconds, kill me if you think it is needed \n");
	sleep(10);
	printf("DOING IT NOW! \n");
	sdh_pos( ql, SDH_LEFT, tsec );
	printf("I AM DONE cleaning up \n");	
	aa_mem_region_local_release();
	printf("Ready to go\n");
    } else {
	std::cout << "No doing nothing!!!"<< std::endl;
    }



    // Wait for 7 seconds 
    printf("About to sleep \n");
    sleep(7);

    // Exit
    printf("I go to sleep, bye! \n");

    // Update constantly
    /*
    while( !sns_cx.shutdown ) {
	update();
	aa_mem_region_local_release();

	sleep(1);
    }
    */

    sns_end();
    return 0;

}

void sdh_pos( double* x,
	      int side,
	      double tsec ) {

    // Time in seconds -> nanoseconds
    double tnano = tsec*1e9;

    if( tnano < 3000000000 ) {
	printf("Time less than 3 sec is too small and dangerous! \n");
	return;
    }

    // Update time
    if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) {
	SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n",
		 strerror(errno) );
    }


    struct sns_msg_motor_ref* msg = sns_msg_motor_ref_local_alloc( SDH_AXES );
    sns_msg_header_fill( &msg->header );
    msg->mode = SNS_MOTOR_MODE_POS;
    AA_MEM_CPY( msg->u, x, SDH_AXES );
    aa_dump_vec( stdout, x, SDH_AXES );


    // Duration from now till now + tnano
    sns_msg_set_time( &msg->header, &now, tnano );

    switch( side ) {
    case SDH_LEFT: {
	ach_put( &chan_sdhref_left, msg, sns_msg_motor_ref_size(msg) );
	break; 
    }
    case SDH_RIGHT: {
	ach_put( &chan_sdhref_right, msg, sns_msg_motor_ref_size(msg) );
	break;
    }
    } // end switch

} 

/**
 * @function update
 */
static void update( void ) {

    if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) {
	SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n",
		 strerror(errno) );
    }

    struct timespec timeout = sns_time_add_ns( now, 1000*1000*1 );
    int is_updated = 0;

    // Get SDH
    int u_sl = update_n( SDH_AXES, ql, dql, 
			 &chan_sdhstate_left,
			 &timeout );

    printf("SDH Left: Pos: (");
    for( int i = 0; i < SDH_AXES; ++i ) {
	printf(" %f ", ql[i] );
    } printf("\n");
    printf("SDH Right: Vel: (");
    for( int i = 0; i < SDH_AXES; ++i ) {
	printf(" %f ", dql[i] );
    } printf("\n");


    int u_sr = update_n( SDH_AXES, qr, dqr,
			 &chan_sdhstate_right,
			 &timeout );

    printf("SDH Right: Pos: (");
    for( int i = 0; i < SDH_AXES; ++i ) {
	printf(" %f ", qr[i] );
    } printf("\n");
    printf("SDH Right: Vel: (");
    for( int i = 0; i < SDH_AXES; ++i ) {
	printf(" %f ", dqr[i] );
    } printf("\n");
    
    is_updated = is_updated || u_sl || u_sr;
			 
}

static int update_n( size_t n,
		     double* q, double *dq,
		     ach_channel_t *chan,
		     struct timespec *ts ) {

    size_t frame_size;
    void *buf = NULL;
    ach_status_t r = sns_msg_local_get( chan, &buf, 
					&frame_size,
					ts, ACH_O_LAST | (ts ? ACH_O_WAIT : 0 ) );
    
    switch(r) {
    case ACH_OK:
    case ACH_MISSED_FRAME:
	{
	    struct sns_msg_motor_state *msg = (struct sns_msg_motor_state*)buf;
	    if( n == msg->header.n &&
		frame_size == sns_msg_motor_state_size_n((uint32_t)n) ) {
		for( size_t j = 0; j < n; ++j ) {
		    q[j] = msg->X[j].pos;
		    dq[j] = msg->X[j].vel;
		} // end for
		return 1;
	    } // end if
	    else {
		SNS_LOG( LOG_ERR, "Invalid motor_state message \n" );
	    }
	} break;

    case ACH_TIMEOUT:
    case ACH_STALE_FRAMES:
    case ACH_CANCELED:
	break;
    default:
	{SNS_LOG( LOG_ERR, "Failed ach_get: %s \n", ach_result_to_string(r) ); }
    } // end switch

    return 0;

}
