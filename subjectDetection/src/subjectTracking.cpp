/**
 * @file subjectTracking.cpp
 */
#include <subjectTracking.h>

#include <iostream>

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

XnBool subjectTracking::mbNeedPose;
XnChar subjectTracking::mstrPose[20] = "";

std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > subjectTracking::mErrors;


subjectTracking::subjectTracking() {

}

subjectTracking::~subjectTracking() {

}


bool subjectTracking::init() {

  // 0. Initialize default values
  mbNeedPose = FALSE;

  nRetVal = XN_STATUS_OK;
  

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


  mUserGenerator.GetSkeletonCap().SetSkeletonProfile( XN_SKEL_PROFILE_ALL );
  
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
  
  // Draw
  XnUserID aUsers[15];
  XnUInt16 nUsers = 15;

  mUserGenerator.GetUsers( aUsers, nUsers );
  for( int i = 0; i < nUsers; ++i ) {
    XnPoint3D com;
    mUserGenerator.GetCoM( aUsers[i], com );
    mDepthGenerator.ConvertRealWorldToProjective( 1, &com, &com );

    // Get joints
    getJoint( aUsers[i], XN_SKEL_HEAD );

    getJoint( aUsers[i], XN_SKEL_NECK );
    getJoint( aUsers[i], XN_SKEL_TORSO );
    getJoint( aUsers[i], XN_SKEL_WAIST );

    getJoint( aUsers[i], XN_SKEL_LEFT_SHOULDER );
    getJoint( aUsers[i], XN_SKEL_LEFT_ELBOW );
  }


}

/**
 * @function getJoint
 */
void subjectTracking::getJoint( XnUserID player,
				XnSkeletonJoint eJoint ) {

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
  std::cout << "Joint position ("<< eJoint<<"): "<< pt.X<<", "<<pt.Y<<", "<< pt.Z << std::endl;

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



void XN_CALLBACK_TYPE subjectTracking::user_new( xn::UserGenerator & /* generator*/,
						 XnUserID nId,
						 void* /* pCookie */) {

  XnUInt32 epochTime = 0;
  xnOSGetEpochTime( &epochTime );
  printf( "[Epoch time: %d] New User %d \n", epochTime, nId);
  
  // New user was found 
  if( mbNeedPose ) {
    std::cout << "Need pose for calibration in user new..."<< std::endl;
    mUserGenerator.GetPoseDetectionCap().StartPoseDetection( mstrPose, nId );
  } else {
    mUserGenerator.GetSkeletonCap().RequestCalibration( nId, TRUE );
  }
  

}

void XN_CALLBACK_TYPE subjectTracking::user_lost( xn::UserGenerator & /* generator */,
						  XnUserID nId,
						  void* /** pCookie*/ ) {

  XnUInt32 epochTime = 0;
  xnOSGetEpochTime(&epochTime);
  printf( "[Epoch Time: %d] Lost User: %d \n", epochTime, nId );

}


void XN_CALLBACK_TYPE subjectTracking::poseDetected( xn::PoseDetectionCapability & /* capability */,
						     const XnChar* strPose,
						     XnUserID nId,
						     void* /* pCookie */ ) {
  

  std::cout << "Pose detected!"<< std::endl;
  XnUInt32 epochTime = 0;
  xnOSGetEpochTime( &epochTime );
  printf("[Epoch Time: %d] Pose %s detected for user %d \n", epochTime, strPose, nId );

  mUserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
  mUserGenerator.GetSkeletonCap().RequestCalibration( nId, TRUE );


}
  
void XN_CALLBACK_TYPE subjectTracking::calibStart( xn::SkeletonCapability & /* capability */,
						   XnUserID nId,
						   void* /* pCookie */ ) {

  XnUInt32 epochTime = 0;
  xnOSGetEpochTime( &epochTime );
  printf("[Epoch Time: %d] Calibration started for user %d \n", epochTime,  nId );

}

  
void XN_CALLBACK_TYPE subjectTracking::calibComplete( xn::SkeletonCapability & /* capability */,
						      XnUserID nId,
						      XnCalibrationStatus eStatus,
						      void* /* pCookie */ ) {

  XnUInt32 epochTime = 0;
  xnOSGetEpochTime( &epochTime );

  // Calibration succeeded
  if( eStatus == XN_CALIBRATION_STATUS_OK ) {
    printf("[Epoch time: %d] Calibration complte, start tracking user %d \n", epochTime, nId );
    mUserGenerator.GetSkeletonCap().StartTracking(nId);
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



void XN_CALLBACK_TYPE subjectTracking::calibInProgress( xn::SkeletonCapability &_cap,
							XnUserID id,
							XnCalibrationStatus calibError,
							void* pCookie ){

  mErrors[id].first = calibError;
}

void XN_CALLBACK_TYPE subjectTracking::poseInProgress( xn::PoseDetectionCapability &_cap,
						       const XnChar* strPose,
						       XnUserID id,
						       XnPoseDetectionStatus poseError,
						       void* pCookie ){

  mErrors[id].second = poseError;

}
