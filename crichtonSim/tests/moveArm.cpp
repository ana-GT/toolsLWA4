
#include <time.h>
#include <sys/stat.h>
#include <stdint.h>
#include <unistd.h>
#include <ach.h>
#include<sns.h>


typedef struct {
    ach_channel_t chan_ref_left;
    ach_channel_t chan_ref_right;

    struct sns_msg_motor_ref *msg_ref_left;
    struct sns_msg_motor_ref *msg_ref_right;
} cx_t;

int main( int argc, char* argv[] ) {
    
    cx_t mCx; int r;

    // Open channels
    r = ach_open( &mCx.chan_ref_left, "ref-left", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening left arm ref chan"<<std::endl;
	return 1; }
    
    r = ach_open( &mCx.chan_ref_right, "ref-right", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening right arm ref chan"<<std::endl;
	return 1; }

    mCx.msg_ref_left = sns_msg_motor_ref_heap_alloc( 7 );
    mCx.msg_ref_right = sns_msg_motor_ref_heap_alloc( 7 );

    // Put some constant, slow velocity in the messages
    mCx.msg_ref_left->mode = SNS_MOTOR_MODE_VEL;
    mCx.msg_ref_right->mode = SNS_MOTOR_MODE_VEL;
    for( int i = 0; i < 7; ++i ) {
	mCx.msg_ref_left->u[i] = 0.08;
	mCx.msg_ref_right->u[i] = 0.08;
    }


    // Keep sending
    while(true) {
	ach_put( &mCx.chan_ref_left,
		 mCx.msg_ref_left,
		 sns_msg_motor_ref_size(mCx.msg_ref_left) );
	
	usleep(25*1000);
    }


}
