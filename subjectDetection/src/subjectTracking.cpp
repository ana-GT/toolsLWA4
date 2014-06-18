/**
 * @file subjectTracking.cpp
 */
#include <subjectTracking.h>

#include <iostream>


/** (Global, yeah, yeah) Variables */
xn::Context subjectTracking::mContext;
xn::ScriptNode subjectTracking::mScriptNode;
xn::DepthGenerator subjectTracking::mDepthGenerator;
xn::UserGenerator subjectTracking::mUserGenerator;
xn::Player subjectTracking::mPlayer;

XnCallbackHandle subjectTracking::mhUserCallbacks;
XnCallbackHandle subjectTracking::mhCalibStart;
XnCallbackHandle subjectTracking::mhCalibComplete;
XnCallbackHandle subjectTracking::mhPoseDetected;
XnCallbackHandle subjectTracking::mhCalibInProgress;
XnCallbackHandle subjectTracking::mhPoseInProgress;

bool subjectTracking::mUseCalibFile_flag;
char* subjectTracking::mCalib_filename;
bool subjectTracking::mSaveCalibFile_flag;


XnBool subjectTracking::mbNeedPose = FALSE;
XnChar subjectTracking::mstrPose[20] = "";

std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > subjectTracking::mErrors;

/**
 * @function subjectTrackig
 * @brief Constructor
 */
subjectTracking::subjectTracking() {

}


/**
 * @function subjectTrackig
 * @brief Destructor
 */
subjectTracking::~subjectTracking() {

}


bool subjectTracking::init() {
  std::cout << "Init "<< std::endl;
  // -1. Initialize default values
  mbNeedPose = FALSE;

  nRetVal = XN_STATUS_OK;
  
  mUseCalibFile_flag = false;
  mSaveCalibFile_flag = false;
  mCalib_filename = NULL;

  // 0. Open ach channels - CONSIDER ONE USER ONLY
  enum ach_status r;
  r = ach_open( &mJointsUser.left_arm_chan, "subject_larm_state", NULL );
  if( r != ACH_OK ) { printf("Subject left arm channel open error \n"); return false; }

  r = ach_open( &mJointsUser.right_arm_chan, "subject_rarm_state", NULL );
  if( r != ACH_OK ) { printf("Subject right arm channel open error \n"); return false; }

  r = ach_open( &mJointsUser.upper_body_chan, "subject_upper_state", NULL );
  if( r != ACH_OK ) { printf("Subject upper body channel open error \n"); return false; }



  // 1. Initialize context object
  nRetVal = mContext.Init();
  
  if( nRetVal != XN_STATUS_OK ) {
    printf("\t Something went wrong initializing context. Exiting \n");
    return 1;
  }

  // 2. Creating a depth generator
  nRetVal = mDepthGenerator.Create( mContext );
  if( nRetVal != XN_STATUS_OK ) {
    printf( "\t [ERROR] We could not create a depth generator \n" );
    return 1;
  }

  // 3. Creating a user generator
  nRetVal = mUserGenerator.Create( mContext );
  if( nRetVal != XN_STATUS_OK ) {
    printf("\t [ERROR] We could not create a user generator \n");
    return 1;
  }

  // 4. Checking if skeleton is supported
  if( !mUserGenerator.IsCapabilitySupported( XN_CAPABILITY_SKELETON ) ) {
    printf("\t [DEFICIENCY] Supplied user generator does not support skeleton! \n");
    return 1;
  }

  // 5. Register callbacks for new/lost user
  nRetVal = mUserGenerator.RegisterUserCallbacks( user_new,
						  user_lost,
						  NULL,
						  mhUserCallbacks );
					       
  if( nRetVal != XN_STATUS_OK ) {
    printf("\t [ERROR] Could not register user callbacks for user generator! \n");
    return 1;
  }

  // 6. Register calibration start/goal functions
  nRetVal = mUserGenerator.GetSkeletonCap().RegisterToCalibrationStart( calibStart,
									NULL,
									mhCalibStart );
  if( nRetVal != XN_STATUS_OK ) {
    printf("\t [ERROR] Could not register Calib start for user generator! \n");
    return 1;
  }


  nRetVal = mUserGenerator.GetSkeletonCap().RegisterToCalibrationComplete( calibComplete,
									   NULL,
									   mhCalibComplete );
  if( nRetVal != XN_STATUS_OK ) {
    printf("\t [ERROR] Could not register Calib complete for user generator! \n");
    return 1;
  }
  

  if( mUserGenerator.GetSkeletonCap().NeedPoseForCalibration() ) {
    mbNeedPose = TRUE;
    printf("\t [INFO] Setting need pose to true!! \n");
    if( !mUserGenerator.IsCapabilitySupported( XN_CAPABILITY_POSE_DETECTION) ) {
      printf("Pose required for calibration, but not supported \n");
      return 1;
    }

    nRetVal = mUserGenerator.GetPoseDetectionCap().RegisterToPoseDetected( poseDetected,
									   NULL,
									   mhPoseDetected );
    if( nRetVal != XN_STATUS_OK ) {
      printf("\t [ERROR] Could not register pose detected callback function \n" );
      return 1;
    }
    printf("Calibration pose before: %s \n", mstrPose);    
    mUserGenerator.GetSkeletonCap().GetCalibrationPose(mstrPose);
    printf("Calibration pose: %s \n", mstrPose);

    
    nRetVal = mUserGenerator.GetPoseDetectionCap().RegisterToPoseInProgress( poseInProgress,
									     NULL,
									     mhPoseInProgress );
    if( nRetVal != XN_STATUS_OK ) {
      printf("\t [ERROR] Could not register callback for pose in progress \n");
      return 1;
    }
    
  }


  mUserGenerator.GetSkeletonCap().SetSkeletonProfile( XN_SKEL_PROFILE_UPPER );
  
  nRetVal = mUserGenerator.GetSkeletonCap().RegisterToCalibrationInProgress( calibInProgress,
									     NULL, 
									     mhCalibInProgress );
  if( nRetVal != XN_STATUS_OK ) {
    printf("\t [ERROR] Could not register callback to calibration in progress \n");
    return 1;
  }
    

  nRetVal = mContext.StartGeneratingAll();
  
  if( nRetVal != XN_STATUS_OK ) {
    printf("\t [ERROR] Could not start generating data! \n");
    return 1;
  }
  

}


/**
 * @function update
 */
bool subjectTracking::update(){

  xn::SceneMetaData sceneMD;
  xn::DepthMetaData depthMD;

  // Grab new information
  mDepthGenerator.GetMetaData( depthMD );

  // Read next available data
  mContext.WaitOneUpdateAll( mUserGenerator );
  
  // Process the data
  mDepthGenerator.GetMetaData( depthMD );
  mUserGenerator.GetUserPixels(0,sceneMD );
  
  // Get joints of all users at this current moment and send them if user is being tracked
  updateJointsAll();
  
}

/**
 * @function sendJointsAll
 */
void subjectTracking::sendJointsAll() {
  
  enum ach_status r; 

  r = ach_put( &mJointsUser.left_arm_chan, 
	       mJointsUser.left_arm_q, 
	       sizeof( mJointsUser.left_arm_q) );
  if( r != ACH_OK ) {
    printf("Could not send left arm msg: %s \n", ach_result_to_string(r) );
  }

  r = ach_put( &mJointsUser.right_arm_chan, 
	       mJointsUser.right_arm_q, 
	       sizeof( mJointsUser.right_arm_q) );
  if( r != ACH_OK ) {
    printf("Could not send right arm msg: %s \n", ach_result_to_string(r) );
  }

  r = ach_put( &mJointsUser.upper_body_chan, 
	       mJointsUser.upper_body_q, 
	       sizeof( mJointsUser.upper_body_q) );
  if( r != ACH_OK ) {
    printf("Could not send upper body msg: %s \n", ach_result_to_string(r) );
  }


}


/**
 * @function updateJointsAll
 */

void subjectTracking::updateJointsAll() {

  XnUserID aUsers[15];
  XnUInt16 nUsers = 15;

  mUserGenerator.GetUsers( aUsers, nUsers );

  if( nUsers > 1 ) {
    printf(" WATCH OUT! More than one user detected. THIS CAN BE MESSY \n");
  }

  for( int i = 0; i < nUsers; ++i ) {
    
    if( mUserGenerator.GetSkeletonCap().IsTracking( aUsers[i] ) ) {
      updateJointsUser( aUsers[i] );

      // Send message with joints information
      sendJointsAll();
    }
        
  } 

}

/**
 * @function updateJointUser
 * @brief Fill joint information (ASSUME USER : 1 ONLY PERSON)
 */
void subjectTracking::updateJointsUser( XnUserID player ) {

  getJoint( player, XN_SKEL_HEAD,
	    mJointsUser.upper_body_q[0] );    
  getJoint( player, XN_SKEL_NECK,
	    mJointsUser.upper_body_q[1] );
  getJoint( player, XN_SKEL_TORSO,
	    mJointsUser.upper_body_q[2] );
    
  getJoint( player, XN_SKEL_LEFT_SHOULDER,
	    mJointsUser.left_arm_q[0] );
  getJoint( player, XN_SKEL_LEFT_ELBOW,
	    mJointsUser.left_arm_q[1] );
  getJoint( player, XN_SKEL_LEFT_HAND,
	    mJointsUser.left_arm_q[2] );
  
  getJoint( player, XN_SKEL_RIGHT_SHOULDER,
	    mJointsUser.right_arm_q[0] );
  getJoint( player, XN_SKEL_RIGHT_ELBOW,
	    mJointsUser.right_arm_q[1] );
  getJoint( player, XN_SKEL_RIGHT_HAND,
	    mJointsUser.right_arm_q[2] );
  
}



void subjectTracking::getJoint( XnUserID player,
				XnSkeletonJoint eJoint,
				double jointPos[3] ) {

  if( !mUserGenerator.GetSkeletonCap().IsTracking( player ) ) {
    printf("User %d not tracked \n", player );
  }
  
  if( !mUserGenerator.GetSkeletonCap().IsJointActive(eJoint) ) { 
    return; 
  }

  XnSkeletonJointPosition joint;
  mUserGenerator.GetSkeletonCap().GetSkeletonJointPosition( player, eJoint, joint );
  if( joint.fConfidence < 0.5 ) {
    return;
  }

  XnPoint3D pt;
  pt = joint.position;
  jointPos[0] = pt.X;
  jointPos[1] = pt.Y;
  jointPos[2] = pt.Z;
}


/**
 * @function CleanupExit
 * @brief Clean nodes / generators 
 */
void subjectTracking::CleanupExit() {

  mScriptNode.Release();
  mDepthGenerator.Release();
  mUserGenerator.Release();
  mPlayer.Release();
  mContext.Release();

  exit(1);
}


/**
 * @function user_new
 * @brief Callback for when new user is detected
 */
void XN_CALLBACK_TYPE subjectTracking::user_new( xn::UserGenerator & /* generator*/,
						 XnUserID nId,
						 void* /* pCookie */) {

  XnUInt32 epochTime = 0;
  xnOSGetEpochTime( &epochTime );
  printf( "[Epoch time: %d] New User %d \n", epochTime, nId);
  
  // New user was found 
  if( mbNeedPose ) {
    mUserGenerator.GetPoseDetectionCap().StartPoseDetection( mstrPose, nId );
  } else {
    mUserGenerator.GetSkeletonCap().RequestCalibration( nId, TRUE );
  }
  

}

/**
 * @function user_lost
 * @brief Callback for when existing user is lost 
 */
void XN_CALLBACK_TYPE subjectTracking::user_lost( xn::UserGenerator & /* generator */,
						  XnUserID nId,
						  void* /** pCookie*/ ) {

  XnUInt32 epochTime = 0;
  xnOSGetEpochTime(&epochTime);
  printf( "[Epoch Time: %d] Lost User: %d \n", epochTime, nId );

}

/**
 * @function poseDetected
 * @brief Called when a pose is detected
 */
void XN_CALLBACK_TYPE subjectTracking::poseDetected( xn::PoseDetectionCapability & /* capability */,
						     const XnChar* strPose,
						     XnUserID nId,
						     void* /* pCookie */ ) {
  

  XnUInt32 epochTime = 0;
  xnOSGetEpochTime( &epochTime );
  printf("[Epoch Time: %d] Pose %s detected for user %d \n", epochTime, strPose, nId );

  mUserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);  

  if( !mUseCalibFile_flag ) {
    mUserGenerator.GetSkeletonCap().RequestCalibration( nId, TRUE );  
  }
  else {
    XnStatus rc = mUserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile( nId,
									       mCalib_filename );
    if( rc == XN_STATUS_OK ) {
      mUserGenerator.GetPoseDetectionCap().StopPoseDetection( nId );
      mUserGenerator.GetSkeletonCap().StartTracking( nId );
    } else {
      printf("\t [ERROR] Calibration file did NOT Load fine! \n");
    }
  } 



}
  
/**
 * @function calibStart
 * @brief Callback when calibration is started
 */
void XN_CALLBACK_TYPE subjectTracking::calibStart( xn::SkeletonCapability & /* capability */,
						   XnUserID nId,
						   void* /* pCookie */ ) {

  XnUInt32 epochTime = 0;
  xnOSGetEpochTime( &epochTime );
  printf("[Epoch Time: %d] Calibration started for user %d \n", epochTime,  nId );

}



/**
 * @function calibComplete
 * @brief Callback for when calibration is complete
 */  
void XN_CALLBACK_TYPE subjectTracking::calibComplete( xn::SkeletonCapability & /* capability */,
						      XnUserID nId,
						      XnCalibrationStatus eStatus,
						      void* /* pCookie */ ) {

  XnUInt32 epochTime = 0;
  xnOSGetEpochTime( &epochTime );

  // Calibration succeeded
  if( eStatus == XN_CALIBRATION_STATUS_OK ) {
    printf("[Epoch time: %d] Calibration complete, start tracking user %d \n", epochTime, nId );
    mUserGenerator.GetSkeletonCap().StartTracking(nId);

    // Store calibration information
    if( mSaveCalibFile_flag ) {
      char filename[30];
      sprintf( filename, "user_%d_calibration.bin", nId);
      mUserGenerator.GetSkeletonCap().SaveCalibrationDataToFile( nId,
								 filename );

    }

  }

  // Calibration failed
  else {
    printf("[Epoch Time: %d] Calibration failed for user %d \n", epochTime, nId );

    if( eStatus == XN_CALIBRATION_STATUS_MANUAL_ABORT ) {
      printf("Manual abort occurred, stop attempting to calibrate \n");
      return;
    }

    // Try to use pose
    if( mbNeedPose ) {
      printf("Calib failed, let's try pose detection \n");
      mUserGenerator.GetPoseDetectionCap().StartPoseDetection( mstrPose, nId );
    } else {
      mUserGenerator.GetSkeletonCap().RequestCalibration( nId, TRUE );
    }

  }

  
}


/**
 * @function calibInProgress
 * @brief Callback function called while the calibration is taking place
 */
void XN_CALLBACK_TYPE subjectTracking::calibInProgress( xn::SkeletonCapability &_cap,
							XnUserID id,
							XnCalibrationStatus calibError,
							void* pCookie ){
  mErrors[id].first = calibError;
}

/**
 * @function poseInProgress
 * @brief Callback function called while the pose detection is going on
 */
void XN_CALLBACK_TYPE subjectTracking::poseInProgress( xn::PoseDetectionCapability &_cap,
						       const XnChar* strPose,
						       XnUserID id,
						       XnPoseDetectionStatus poseError,
						       void* pCookie ){

  mErrors[id].second = poseError;

}
