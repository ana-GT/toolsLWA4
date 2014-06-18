/**
 * @file subjectTracking.h
 */
#pragma once

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>

#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <ach.h>

#include <map>



struct joint_user_t {

  double left_arm_q[3][3]; // shoulder(3), elbow(1), hand(3)
  double right_arm_q[3][3];
  double upper_body_q[3][3]; // torso, neck, head

  ach_channel_t left_arm_chan;
  ach_channel_t right_arm_chan;
  ach_channel_t upper_body_chan;
};

enum sides_t {
  LEFT = 0,
  RIGHT
};


/**
 * @class subjectTracking
 */
class subjectTracking {

 public:
  subjectTracking();
  ~subjectTracking();

  bool init();
  bool update();

  void CleanupExit();
  

 private:

  static void XN_CALLBACK_TYPE user_new( xn::UserGenerator & /* generator*/,
					 XnUserID nId,
					 void* /* pCookie */);
  static void XN_CALLBACK_TYPE user_lost( xn::UserGenerator & /* generator */,
				   XnUserID nId,
				   void* /** pCookie*/ );

  static void XN_CALLBACK_TYPE poseDetected( xn::PoseDetectionCapability & /* capability */,
					     const XnChar* strPose,
					     XnUserID nId,
					     void* /* pCookie */ );
  
  static void XN_CALLBACK_TYPE calibStart( xn::SkeletonCapability & /* capability */,
					   XnUserID nId,
					   void* /* pCookie */ );
  
  static void XN_CALLBACK_TYPE calibComplete( xn::SkeletonCapability & /* capability */,
					      XnUserID nId,
					      XnCalibrationStatus eStatus,
					      void* /* pCookie */ );
  
  void updateJointsAll();
  void sendJointsAll();
  void updateJointsUser( XnUserID player );
  void getJoint( XnUserID player,
		 XnSkeletonJoint eJoint,
		 double jointPos[3] );

  
  static void XN_CALLBACK_TYPE calibInProgress( xn::SkeletonCapability &_cap,
						XnUserID id,
						XnCalibrationStatus calibError,
						void* pCookie );

  static void XN_CALLBACK_TYPE poseInProgress( xn::PoseDetectionCapability &_cap,
					       const XnChar* strPose,
					       XnUserID id,
					       XnPoseDetectionStatus poseError,
					       void* pCookie );

  
  joint_user_t mJointsUser;

  // Debug
  XnStatus nRetVal;

  public:

  static xn::Context mContext;
  static xn::ScriptNode mScriptNode;
  static xn::DepthGenerator mDepthGenerator;
  static xn::UserGenerator mUserGenerator;
  static xn::Player mPlayer;

  static XnCallbackHandle mhUserCallbacks;
  static XnCallbackHandle mhCalibStart;
  static XnCallbackHandle mhCalibComplete;
  static XnCallbackHandle mhPoseDetected;
  static XnCallbackHandle mhCalibInProgress;
  static XnCallbackHandle mhPoseInProgress;

  static XnBool mbNeedPose;
  static XnChar mstrPose[20];
  
  static bool mUseCalibFile_flag;
  static char* mCalib_filename;
  static bool mSaveCalibFile_flag;

  static std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > mErrors;

};
