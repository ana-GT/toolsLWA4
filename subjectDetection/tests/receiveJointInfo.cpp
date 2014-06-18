
#include <unistd.h>
#include <sys/stat.h>
#include <time.h>
#include <stdint.h>
#include <ach.h>
#include <stdio.h>

int main( int argc, char* argv[] ) {


  ach_channel_t left_arm_chan;
  ach_channel_t right_arm_chan;
  ach_channel_t upper_body_chan;

  double left_arm_q[3][3];
  double right_arm_q[3][3];
  double upper_body_q[3][3];

  // Open ach channels
  enum ach_status r;
  r = ach_open( &left_arm_chan, "subject_larm_state", NULL );
  if( r != ACH_OK ) {
    printf("Could not open left arm correctly \n");
  }

  r = ach_open( &right_arm_chan, "subject_rarm_state", NULL );
  if( r != ACH_OK ) {
    printf("Could not open right arm correctly \n");
  }


  r = ach_open( &upper_body_chan, "subject_upper_state", NULL );
  if( r != ACH_OK ) {
    printf("Could not open upper body correctly \n");
  }


  // Read constantly
  size_t frame_size;

  while( true ) {

    // Left Arm
    ach_get( &left_arm_chan, &left_arm_q,sizeof( left_arm_q ),
	     &frame_size, NULL, 0 );
    if( ACH_MISSED_FRAME == r ) {
      printf("Missed some messages \n");
    } else if( ACH_STALE_FRAMES == r ) {
      printf("No new data \n");
    } else if( ACH_OK != r ) {
      printf("Unable to get a message. \n");
    } else if( frame_size != sizeof(left_arm_q) ) {
      printf("Unexpected message of size %d , we were expecting size %d \n", 
	     frame_size, sizeof(left_arm_q) );
    }

    printf("*********************\n");
    printf("Left shoulder: %f %f %f \n", left_arm_q[0][0], left_arm_q[0][1], left_arm_q[0][2] );
    printf("Left elbow: %f %f %f \n", left_arm_q[1][0], left_arm_q[1][1], left_arm_q[1][2] );
    printf("Left wrist: %f %f %f \n", left_arm_q[2][0], left_arm_q[2][1], left_arm_q[2][2] );

    printf("  -- ** -- \n");

    // Right Arm
    ach_get( &right_arm_chan, &right_arm_q,sizeof( right_arm_q ),
	     &frame_size, NULL, 0 );
    if( ACH_MISSED_FRAME == r ) {
      printf("Missed some messages \n");
    } else if( ACH_STALE_FRAMES == r ) {
      printf("No new data \n");
    } else if( ACH_OK != r ) {
      printf("Unable to get a message. \n");
    } else if( frame_size != sizeof(right_arm_q) ) {
      printf("Unexpected message of size %d , we were expecting size %d \n", 
	     frame_size, sizeof(right_arm_q) );
    }

    printf("Right shoulder: %f %f %f \n", right_arm_q[0][0], right_arm_q[0][1], right_arm_q[0][2] );
    printf("Right elbow: %f %f %f \n", right_arm_q[1][0], right_arm_q[1][1], right_arm_q[1][2] );
    printf("Right wrist: %f %f %f \n", right_arm_q[2][0], right_arm_q[2][1], right_arm_q[2][2] );

    printf("  -- ** -- \n");

    // Upper body
    ach_get( &upper_body_chan, &upper_body_q,sizeof( upper_body_q ),
	     &frame_size, NULL, 0 );
    if( ACH_MISSED_FRAME == r ) {
      printf("Missed some messages \n");
    } else if( ACH_STALE_FRAMES == r ) {
      printf("No new data \n");
    } else if( ACH_OK != r ) {
      printf("Unable to get a message. \n");
    } else if( frame_size != sizeof(upper_body_q) ) {
      printf("Unexpected message of size %d , we were expecting size %d \n", 
	     frame_size, sizeof(upper_body_q) );
    }

    printf("Head: %f %f %f \n", upper_body_q[0][0], upper_body_q[0][1], upper_body_q[0][2] );
    printf("Neck: %f %f %f \n", upper_body_q[1][0], upper_body_q[1][1], upper_body_q[1][2] );
    printf("Torso: %f %f %f \n", upper_body_q[2][0], upper_body_q[2][1], upper_body_q[2][2] );

    printf("*********************\n");

    usleep(200*1000);

  }

  return 0;
}
