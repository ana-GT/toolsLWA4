/**
 * @file crichtonSim.h
 */
#pragma once

#include "crichtonSim.h"

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


/**
 * @class crichtonSim
 */
class crichtonSim {

 public:
  
  crichtonSim();
  ~crichtonSim();
  
  bool setRobot( dart::dynamics::Skeleton* _leftArm,
		 dart::dynamics::Skeleton* _rightArm,
		 dart::dynamics::Skeleton* _leftHand,
		 dart::dynamics::Skeleton* _rightHand );

  dart::dynamics::Skeleton* getArm( const int &_side );
  dart::dynamics::Skeleton* getHand( const int &_side );

 private:

  dart::dynamics::Skeleton* mRobot;
  dart::dynamics::Skeleton* mLeftArm;
  dart::dynamics::Skeleton* mRightArm;

  dart::dynamics::Skeleton* mLeftHand;
  dart::dynamics::Skeleton* mRightHand;


};
