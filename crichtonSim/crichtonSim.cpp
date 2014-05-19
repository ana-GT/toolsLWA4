/**
 * @file crichtonSim
 */
#include "crichtonSim.h"
#include <dart/dynamics/Skeleton.h>

/**
 * @function crichtonSim
 * @brief Constructor
 */
crichtonSim::crichtonSim() {

}


/**
 * @function ~crichtonSim
 * @brief Destructor
 */
crichtonSim::~crichtonSim() {

}
  
/**
 * @function setRobot
 * @brief Set arms and hands respectively
 */
bool crichtonSim::setRobot( dart::dynamics::Skeleton* _leftArm,
			    dart::dynamics::Skeleton* _rightArm,
			    dart::dynamics::Skeleton* _leftHand,
			    dart::dynamics::Skeleton* _rightHand ) {

  mLeftArm = _leftArm;
  mRightArm = _rightArm;
  mLeftHand = _leftHand;
  mRightHand = _rightHand;
}

/**
 * @function getArm
 */
dart::dynamics::Skeleton* crichtonSim::getArm( const int &_side ) {
  if( _side == LEFT ) { return mLeftArm; }
  else if( _side == RIGHT ) { return mRightArm; }
  else { std::cout <<" [ERROR] Only left or right arm. Return NULL!"<< std::endl; return NULL; }
}

/**
 * @function getHand
 */
dart::dynamics::Skeleton* crichtonSim::getHand( const int &_side ) {

  if( _side == LEFT ) { return mLeftHand; }
  else if( _side == RIGHT ) { return mRightHand; }
  else { std::cout <<" [ERROR] Only left or right hand. Return NULL!"<< std::endl; return NULL; }


}


