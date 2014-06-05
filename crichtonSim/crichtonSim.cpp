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
 * @function initSetup_channels
 * @brief Create ACH channels and open them, letting them ready to send stuff
 */
bool crichtonSim::initSetup_channels() {
    
    ach_status r;

    // Create channels (script beforehand)
    
    // Open channels
    r = ach_open( &mCx.chan_state_left, "state-left", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening left arm chan"<<std::endl;
	return false; }

    r = ach_open( &mCx.chan_state_right, "state-right", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening right arm chan"<<std::endl;
	return false; }

    r = ach_open( &mCx.chan_sdhstate_left, "sdhstate-left", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening left hand chan"<<std::endl; 
	return false; }

    r = ach_open( &mCx.chan_sdhstate_right, "sdhstate-right", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening right hand chan"<<std::endl; 
	return false; }


    // Set heap for messages. For Crichton, we know that 7 modules are used for 
    // each arm and hand
    mCx.msg_state_left = sns_msg_motor_state_heap_alloc( 7 );
    mCx.msg_state_right = sns_msg_motor_state_heap_alloc( 7 );

    mCx.msg_sdhstate_left = sns_msg_motor_state_heap_alloc( 7 );
    mCx.msg_sdhstate_right = sns_msg_motor_state_heap_alloc( 7 );

    std::cout << "\t [GOOD] Finished initing ACH stuff in Crichton sim"<< std::endl;
    return true;
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
 * @function update
 */
void crichtonSim::update() {

    this->putStatesFromSim();
}

/**
 * @function putStatesFromSim
 * @brief Get arm/hand states from simulation and publish them in ACH msgs
 */
void crichtonSim::putStatesFromSim() {

    Eigen::VectorXd qla, qra, qlh, qrh;

    // Left arm
    qla = mLeftArm->getConfig();
    for( int i = 0; i < 7; ++i ) {
	mCx.msg_state_left->X[i].pos = qla(i);
	mCx.msg_state_left->X[i].vel = 0;
    }

    ach_put( &mCx.chan_state_left, 
	     mCx.msg_state_left, 
	     sns_msg_motor_state_size(mCx.msg_state_left) );

    // Right arm
    qra = mRightArm->getConfig();
    for( int i = 0; i < 7; ++i ) {
	mCx.msg_state_right->X[i].pos = qra(i);
	mCx.msg_state_right->X[i].vel = 0;
    }

    ach_put( &mCx.chan_state_right, 
	     mCx.msg_state_right, 
	     sns_msg_motor_state_size(mCx.msg_state_right) );


    // Left hand
    qlh = mLeftHand->getConfig();
    for( int i = 0; i < 7; ++i ) {
	mCx.msg_sdhstate_left->X[i].pos = qlh(i);
	mCx.msg_sdhstate_left->X[i].vel = 0;
    }

    ach_put( &mCx.chan_sdhstate_left, 
	     mCx.msg_sdhstate_left, 
	     sns_msg_motor_state_size(mCx.msg_sdhstate_left) );


    // Right hand
    qrh = mRightHand->getConfig();
    for( int i = 0; i < 7; ++i ) {
	mCx.msg_sdhstate_right->X[i].pos = qrh(i);
	mCx.msg_sdhstate_right->X[i].vel = 0;
    }

    ach_put( &mCx.chan_sdhstate_right, 
	     mCx.msg_sdhstate_right, 
	     sns_msg_motor_state_size(mCx.msg_sdhstate_right) );

}

/**
 * @function getStatesToSim
 * @brief Get arm/hand states from ACH msgs and put them in simulation
 */
void crichtonSim::getStatesToSim() {

    Eigen::VectorXd qla(7), qra(7), qlh(8), qrh(8);
    size_t frame_size;
    void *buf = NULL;
    ach_status_t r;

    // Left arm
    struct timespec *ts = NULL;
    r = sns_msg_local_get( &mCx.chan_state_left,
			   &buf, &frame_size,
			   ts, ACH_O_LAST | (ts? ACH_O_WAIT : 0 ) );
    
    switch(r) {
    case ACH_OK:
    case ACH_MISSED_FRAME:
	mCx.msg_state_left = (struct sns_msg_motor_state*)buf;

	if( frame_size == sns_msg_motor_state_size_n(7) ) {
	    for( int i = 0; i < 7; ++i ) {
		qla(i) = mCx.msg_state_left->X[i].pos;
	    }
	    mLeftArm->setConfig(qla);	    
	}
	break;
    }

    // Right arm
    r = sns_msg_local_get( &mCx.chan_state_right,
			   &buf, &frame_size,
			   ts, ACH_O_LAST | (ts? ACH_O_WAIT : 0 ) );
    
    switch(r) {
    case ACH_OK:
    case ACH_MISSED_FRAME:
	mCx.msg_state_right = (struct sns_msg_motor_state*)buf;

	if( frame_size == sns_msg_motor_state_size_n(7) ) {
	    for( int i = 0; i < 7; ++i ) {
		qra(i) = mCx.msg_state_right->X[i].pos;
	    }
	    mRightArm->setConfig(qra);	    
	}
	break;
    }


    // Left hand
    r = sns_msg_local_get( &mCx.chan_sdhstate_left,
			   &buf, &frame_size,
			   ts, ACH_O_LAST | (ts? ACH_O_WAIT : 0 ) );
    
    switch(r) {
    case ACH_OK:
    case ACH_MISSED_FRAME:
	mCx.msg_sdhstate_left = (struct sns_msg_motor_state*)buf;

	if( frame_size == sns_msg_motor_state_size_n(7) ) {
	    for( int i = 0; i < 7; ++i ) {
		qlh(i) = mCx.msg_sdhstate_left->X[i].pos;
	    } qlh(7) = 0;
	    mLeftHand->setConfig(qlh);	    
	}
	break;
    }

    // Right hand
    r = sns_msg_local_get( &mCx.chan_sdhstate_right,
			   &buf, &frame_size,
			   ts, ACH_O_LAST | (ts? ACH_O_WAIT : 0 ) );
    
    switch(r) {
    case ACH_OK:
    case ACH_MISSED_FRAME:
	mCx.msg_sdhstate_right = (struct sns_msg_motor_state*)buf;

	if( frame_size == sns_msg_motor_state_size_n(7) ) {
	    for( int i = 0; i < 7; ++i ) {
		qrh(i) = mCx.msg_sdhstate_right->X[i].pos;
	    } qrh(7) = 0;
	    mRightHand->setConfig(qrh);	    
	}
	break;
    }

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


