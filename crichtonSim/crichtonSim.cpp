/**
 * @file crichtonSim
 * @brief Simulate daemon c402 for arm /hand motors
 */
#include "crichtonSim.h"
#include <dart/dynamics/Skeleton.h>

/**
 * @function crichtonSim
 * @brief Constructor
 */
crichtonSim::crichtonSim() {

    // Some default parameters
    mq_la = Eigen::VectorXd::Zero(7); mdq_la = Eigen::VectorXd::Zero(7);
    mq_ra = Eigen::VectorXd::Zero(7); mdq_ra = Eigen::VectorXd::Zero(7);
    mq_lh = Eigen::VectorXd::Zero(8); mdq_lh = Eigen::VectorXd::Zero(8);
    mq_rh = Eigen::VectorXd::Zero(8); mdq_rh = Eigen::VectorXd::Zero(8);

    mdt = 0.025;
    mMode = SNS_MOTOR_MODE_VEL;

    dFingerDofs.resize(8); for( int i = 0; i < 8; ++i ) { dFingerDofs[i] = i+6; }
    dPoseDofs.resize(6); for( int i = 0; i < 6; ++i ) { dPoseDofs[i] = i; }
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
    
    // Open channels to return motor states
    r = ach_open( &mCx.chan_state_left, "state-left", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening left arm state chan"<<std::endl;
	return false; }

    r = ach_open( &mCx.chan_state_right, "state-right", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening right arm state chan"<<std::endl;
	return false; }

    r = ach_open( &mCx.chan_sdhstate_left, "sdhstate-left", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening left hand state chan"<<std::endl; 
	return false; }

    r = ach_open( &mCx.chan_sdhstate_right, "sdhstate-right", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening right hand state chan"<<std::endl; 
	return false; }

    // Open channels to read commands (ref - and we only read velocity at the moment)
    r = ach_open( &mCx.chan_ref_left, "ref-left", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening left arm ref chan"<<std::endl;
	return false; }

    r = ach_open( &mCx.chan_ref_right, "ref-right", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening right arm ref chan"<<std::endl;
	return false; }

    r = ach_open( &mCx.chan_sdhref_left, "sdhref-left", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening left hand ref chan"<<std::endl; 
	return false; }

    r = ach_open( &mCx.chan_sdhref_right, "sdhref-right", NULL );
    if( r != ACH_OK ) { std::cout<<"\t [ERROR] Opening right hand ref chan"<<std::endl; 
	return false; }


    // Set heap for messages. For Crichton, we know that 7 modules are used for 
    // each arm and hand
    mCx.msg_state_left = sns_msg_motor_state_heap_alloc( 7 );
    mCx.msg_state_right = sns_msg_motor_state_heap_alloc( 7 );

    mCx.msg_sdhstate_left = sns_msg_motor_state_heap_alloc( 7 );
    mCx.msg_sdhstate_right = sns_msg_motor_state_heap_alloc( 7 );

    mCx.msg_ref_left = sns_msg_motor_ref_heap_alloc( 7 );
    mCx.msg_ref_right = sns_msg_motor_ref_heap_alloc( 7 );

    mCx.msg_sdhref_left = sns_msg_motor_ref_heap_alloc( 7 );
    mCx.msg_sdhref_right = sns_msg_motor_ref_heap_alloc( 7 );


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
 * @function run
 * @brief Called each time the timer gets a timeout
 */
void crichtonSim::run() {

    // Simulate the robot executing command
    this->simulate();

    // Send state 
    this->update();

    // Receive control signal
    this->control();
}

/**
 * @function simulate
 */
void crichtonSim::simulate() {

    // Simulate according to the mode
    switch( mMode ) {

    case SNS_MOTOR_MODE_HALT: {

    } break;

    case SNS_MOTOR_MODE_POS : {
	std::cout << "Position mode not supported yet"<< std::endl;
    } break;
 
    case SNS_MOTOR_MODE_VEL : {

	// Left Arm
	mq_la = mLeftArm->getConfig();
	mq_la += mdt*mdq_la;
	mLeftArm->setConfig( mq_la );

	// Right Arm
	mq_ra = mRightArm->getConfig();
	mq_ra += mdt*mdq_ra;
	mRightArm->setConfig( mq_ra );

	// Left Hand
	mq_lh = mLeftHand->getConfig( dFingerDofs );
	mq_lh += mdt*mdq_lh;
	mLeftHand->setConfig( dFingerDofs, mq_lh );

	// Right Hand
	mq_rh = mRightHand->getConfig( dFingerDofs );
	mq_rh += mdt*mdq_rh;
	mRightHand->setConfig( dFingerDofs, mq_rh );

    } break;
    
    case SNS_MOTOR_MODE_TORQ: { 
	std::cout << "Torque mode not supported yet"<< std::endl;
    } break;
    
    }

}

/**
 * @function update
 * @brief Get arm/hand states from simulation and publish them in ACH msgs
 */
void crichtonSim::update() {

    // Left arm
    mq_la = mLeftArm->getConfig();
    for( int i = 0; i < 7; ++i ) {
	mCx.msg_state_left->X[i].pos = mq_la(i);
	mCx.msg_state_left->X[i].vel = mdq_la(i);
    }

    ach_put( &mCx.chan_state_left, 
	     mCx.msg_state_left, 
	     sns_msg_motor_state_size(mCx.msg_state_left) );

    // Right arm
    mq_ra = mRightArm->getConfig();
    for( int i = 0; i < 7; ++i ) {
	mCx.msg_state_right->X[i].pos = mq_ra(i);
	mCx.msg_state_right->X[i].vel = mdq_ra(i);
    }

    ach_put( &mCx.chan_state_right, 
	     mCx.msg_state_right, 
	     sns_msg_motor_state_size(mCx.msg_state_right) );


    // Left hand
    mq_lh = mLeftHand->getConfig( dFingerDofs );
    for( int i = 0; i < 7; ++i ) {
	mCx.msg_sdhstate_left->X[i].pos = mq_lh(i);
	mCx.msg_sdhstate_left->X[i].vel = mdq_lh(i);
    }

    ach_put( &mCx.chan_sdhstate_left, 
	     mCx.msg_sdhstate_left, 
	     sns_msg_motor_state_size(mCx.msg_sdhstate_left) );


    // Right hand
    mq_rh = mRightHand->getConfig( dFingerDofs );
    for( int i = 0; i < 7; ++i ) {
	mCx.msg_sdhstate_right->X[i].pos = mq_rh(i);
	mCx.msg_sdhstate_right->X[i].vel = mdq_rh(i);
    }

    ach_put( &mCx.chan_sdhstate_right, 
	     mCx.msg_sdhstate_right, 
	     sns_msg_motor_state_size(mCx.msg_sdhstate_right) );

}

/**
 * @function control
 * @brief Get arm/hand states from ACH msgs and put them in simulation
 */
void crichtonSim::control() {

    size_t frame_size;
    void *buf = NULL;
    ach_status_t r;

	// Initialize to vel=0
	for( int i = 0; i < 7; ++i ) { mdq_la(i) = 0; }
	for( int i = 0; i < 7; ++i ) { mdq_ra(i) = 0; }

    // Left arm
    struct timespec *ts = NULL;
    r = sns_msg_local_get( &mCx.chan_ref_left,
			   &buf, &frame_size,
			   ts, ACH_O_LAST | (ts? ACH_O_WAIT : 0 ) );

    switch(r) {
    case ACH_OK:
    case ACH_MISSED_FRAME:
	mCx.msg_ref_left = (struct sns_msg_motor_ref*)buf;

	if( mCx.msg_ref_left->mode != SNS_MOTOR_MODE_VEL ) {
	    std::cout <<"\t [ERROR] We only use velocity mode currently!"<< std::endl;
	    return;
	}

	if( frame_size == sns_msg_motor_ref_size_n(7) ) {
	    for( int i = 0; i < 7; ++i ) {
		mdq_la(i) = mCx.msg_ref_left->u[i];
	    }
	}
	break;
    }

    // Right arm
    r = sns_msg_local_get( &mCx.chan_ref_right,
			   &buf, &frame_size,
			   ts, ACH_O_LAST | (ts? ACH_O_WAIT : 0 ) );
    
    switch(r) {
    case ACH_OK:
    case ACH_MISSED_FRAME:
	mCx.msg_ref_right = (struct sns_msg_motor_ref*)buf;

	if( mCx.msg_ref_right->mode != SNS_MOTOR_MODE_VEL ) {
	    std::cout <<"\t [ERROR] We only use velocity mode currently!"<< std::endl;
	    return;
	}

	if( frame_size == sns_msg_motor_ref_size_n(7) ) {
	    for( int i = 0; i < 7; ++i ) {
		mdq_ra(i) = mCx.msg_ref_right->u[i];
	    }
	}
	break;
    }


    // Left hand
    r = sns_msg_local_get( &mCx.chan_sdhref_left,
			   &buf, &frame_size,
			   ts, ACH_O_LAST | (ts? ACH_O_WAIT : 0 ) );
    
    switch(r) {
    case ACH_OK:
    case ACH_MISSED_FRAME:
	mCx.msg_sdhref_left = (struct sns_msg_motor_ref*)buf;

	if( mCx.msg_sdhref_left->mode != SNS_MOTOR_MODE_VEL ) {
	    std::cout <<"\t [ERROR] We only use velocity mode currently!"<< std::endl;
	    return;
	}

	if( frame_size == sns_msg_motor_ref_size_n(7) ) {
	    for( int i = 0; i < 7; ++i ) {
		mdq_lh(i) = mCx.msg_sdhref_left->u[i];
	    } mdq_lh(7) = 0;
	}
	break;
    }

    // Right hand
    r = sns_msg_local_get( &mCx.chan_sdhref_right,
			   &buf, &frame_size,
			   ts, ACH_O_LAST | (ts? ACH_O_WAIT : 0 ) );
    
    switch(r) {
    case ACH_OK:
    case ACH_MISSED_FRAME:
	mCx.msg_sdhref_right = (struct sns_msg_motor_ref*)buf;

	if( mCx.msg_sdhref_right->mode != SNS_MOTOR_MODE_VEL ) {
	    std::cout <<"\t [ERROR] We only use velocity mode currently!"<< std::endl;
	    return;
	}

	if( frame_size == sns_msg_motor_ref_size_n(7) ) {
	    for( int i = 0; i < 7; ++i ) {
		mdq_rh(i) = mCx.msg_sdhref_right->u[i];
	    } mdq_rh(7) = 0;
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


