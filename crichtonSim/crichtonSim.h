/**
 * @file crichtonSim.h
 */
#pragma once

#include <time.h>
#include <sys/stat.h>
#include <stdint.h>
#include <ach.h>
#include <sns.h>

namespace dart{ 
  namespace dynamics {
    class Skeleton;
  }
}

/** Side */
enum {
  LEFT = 0,
  RIGHT
};

typedef struct {

    ach_channel_t chan_state_left;
    ach_channel_t chan_state_right;
    ach_channel_t chan_sdhstate_left;
    ach_channel_t chan_sdhstate_right;
    
    struct sns_msg_motor_state *msg_state_left;
    struct sns_msg_motor_state *msg_state_right;
    struct sns_msg_motor_state *msg_sdhstate_left;
    struct sns_msg_motor_state *msg_sdhstate_right;


} cx_t;

/**
 * @class crichtonSim
 */
class crichtonSim {

 public:
  
  crichtonSim();
  ~crichtonSim();

  bool initSetup_channels();
  
  bool setRobot( dart::dynamics::Skeleton* _leftArm,
		 dart::dynamics::Skeleton* _rightArm,
		 dart::dynamics::Skeleton* _leftHand,
		 dart::dynamics::Skeleton* _rightHand );

  void update();
  void putStatesFromSim();
  void getStatesToSim();

  dart::dynamics::Skeleton* getArm( const int &_side );
  dart::dynamics::Skeleton* getHand( const int &_side );

 private:

  dart::dynamics::Skeleton* mRobot;
  dart::dynamics::Skeleton* mLeftArm;
  dart::dynamics::Skeleton* mRightArm;

  dart::dynamics::Skeleton* mLeftHand;
  dart::dynamics::Skeleton* mRightHand;

  cx_t mCx;


};
