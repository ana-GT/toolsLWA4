/**
 * @file piranha_viz_ui.cpp
 * @brief Read arm/hand states from ACH msgs and shows them in simulation
 */
 
#include "piranha_viz_ui.h"
#include <iostream>
#include <qplugin.h>
#include <QtGui>
#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/Joint.h>
#include <dart/dynamics/BodyNode.h>
#include <dart/math/Geometry.h>

#include <ach.h>

/**
 * @function piranha_viz_ui
 * @brief Constructor
 */
piranha_viz_ui::piranha_viz_ui(QWidget *parent) : ui(new Ui::piranha_viz_ui){
    ui->setupUi(this);

    // Set button connections
    connect( ui->init_ACH_pushButton, SIGNAL(released()), 
	     this, SLOT(init_ACH()) );
    connect( ui->stop_ACH_pushButton, SIGNAL(released()), 
	     this, SLOT(stop_ACH()) );

    connect( ui->initializeSimParams_pushButton, SIGNAL(released()), 
	     this, SLOT(initializeSimParams()) );

    connect( ui->startUpdate_pushButton, SIGNAL(released()), 
	     this, SLOT(startUpdate()) );
    connect( ui->stopUpdate_pushButton, SIGNAL(released()), 
	     this, SLOT(stopUpdate()) );


    connect( ui->init_ACH_subject_pushButton, SIGNAL(released()), 
	     this, SLOT(init_ACH_subject()) );
    connect( ui->startUpdate_subject_pushButton, SIGNAL(released()), 
	     this, SLOT(startUpdate_subject()) );
    connect( ui->stopUpdate_subject_pushButton, SIGNAL(released()), 
	     this, SLOT(stopUpdate_subject()) );

    connect( ui->empty_3_1_pushButton, SIGNAL(released()), 
	     this, SLOT(empty_3_1()) );
    connect( ui->empty_3_2_pushButton, SIGNAL(released()), 
	     this, SLOT(empty_3_2()) );
    connect( ui->empty_3_3_pushButton, SIGNAL(released()), 
	     this, SLOT(empty_3_3()) );

    // Set timer connection
    connect( &mTimer, SIGNAL(timeout()), this, SLOT( update()));
    // Set timer for subject tracking updates
    connect( &mTimer_subject, SIGNAL(timeout()), this, SLOT( update_subject() ) );
}

/**
 * @function ~piranha_viz_ui
 * @brief Destructor
 */
piranha_viz_ui::~piranha_viz_ui(){

}

/**
 * @function startAchComm
 * @brief
 */
void piranha_viz_ui::init_ACH() {
    mCrichton.initSetup_channels();
}

/**
 * @function init_stuff_subject
 */
void piranha_viz_ui::init_stuff_subject() {
  
  Twc = Eigen::Isometry3d::Identity();
  Twc.translation() << -0.3, -0.6, 1.4;
  double sq2 = sqrt(2) / 2.0;
  Twc.linear() << -sq2,0,sq2, sq2,0,sq2, 0,1,0;

}

/**
 * @function
 * @brief
 */
void piranha_viz_ui::stop_ACH() {

}

/**
 * @function initUpdate()
 * @brief Start timer so at each timeout (100 ms) it will call
 * the callback timer function (defined in the constructor as update)
 */
void piranha_viz_ui::startUpdate() {
    mTimer.start( mCrichton.getdt()*1000.0 );
}

/**
 * @function stopUpdate()
 * @brief Stop timer, so it won't call callback (update)
 */
void piranha_viz_ui::stopUpdate() {
  mTimer.stop();
}


/**
 * @function update
 * @brief Callback timer function
 */
void piranha_viz_ui::update() {

    mCrichton.run();
    

}

/**
 * @function initializeSimParams
 * @brief Set the robot to use 
 */
void piranha_viz_ui::initializeSimParams() {

  // 1. Set robot loaded, verify that it is a lwa4
  if( !_world->getSkeleton("leftArm") ) {
    std::cout <<"\t * [ERROR] Did not find left arm!"<< std::endl; return; 
  }

  if( !_world->getSkeleton("rightArm") ) {
    std::cout <<"\t * [ERROR] Did not find right arm!"<< std::endl; return; 
  }

  if( !_world->getSkeleton("leftHand") ) {
    std::cout <<"\t * [ERROR] Did not find left hand!"<< std::endl; return; 
  }

  if( !_world->getSkeleton("rightHand") ) {
    std::cout <<"\t * [ERROR] Did not find right hand!"<< std::endl; return; 
  }

  mCrichton.setRobot( _world->getSkeleton("leftArm"),
		      _world->getSkeleton("rightArm"),
		      _world->getSkeleton("leftHand"),
		      _world->getSkeleton("rightHand") );
  
  std::cout << "\t * [GOOD] Set Crichton correctly \n"<< std::endl;
}

/**
 * @function init_ACH_subject_pushButton
 * @brief
 */
void piranha_viz_ui::init_ACH_subject() {

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



}

/**
 * @function initUpdate()
 * @brief Start timer so at each timeout (100 ms) it will call
 * the callback timer function (defined in the constructor as update)
 */
void piranha_viz_ui::startUpdate_subject() {
  init_stuff_subject();
  mTimer_subject.start( 100 );
}

/**
 * @function stopUpdate()
 * @brief Stop timer, so it won't call callback (update)
 */
void piranha_viz_ui::stopUpdate_subject() {
  mTimer_subject.stop();
}

/**
 * @function update_subject()
 * @brief Grab latest message with tracking values and put it in simulation
 */
void piranha_viz_ui::update_subject() {

  size_t frame_size;

  // Left Arm
  int r;
  r = ach_get( &left_arm_chan, &left_arm_q,sizeof( left_arm_q ),
	       &frame_size, NULL, 0 );
  if( ACH_MISSED_FRAME == r ) {
    printf("Missed some messages \n");
  } else if( ACH_STALE_FRAMES == r ) {
    printf("No new data \n");
  } else if( ACH_OK != r ) {
    printf("Ach is not ok. \n");
  } else if( frame_size != sizeof(left_arm_q) ) {
    printf("Unexpected message of size %d , we were expecting size %d \n", 
	   frame_size, sizeof(left_arm_q) );
  } else if( ACH_OK == r ) {
    printf("Ach is OK left! \n");

    Eigen::Vector3d la; la << left_arm_q[0][0], left_arm_q[0][1], left_arm_q[0][2];
    Eigen::Vector3d le; le << left_arm_q[1][0], left_arm_q[1][1], left_arm_q[1][2];
    Eigen::Vector3d lw; lw << left_arm_q[2][0], left_arm_q[2][1], left_arm_q[2][2];
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = tf.linear()*la / 1000.0;
    _world->getSkeleton("lshoulder")->setConfig( dart::math::logMap(Twc*tf) );    
    tf.translation() = tf.linear()*le / 1000.0;
    _world->getSkeleton("lelbow")->setConfig( dart::math::logMap(Twc*tf) );
    tf.translation() = tf.linear()*lw / 1000.0;
    _world->getSkeleton("lhand")->setConfig( dart::math::logMap(Twc*tf) );
  }
  
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
  } else if( ACH_OK == r ) {
    printf("Ach is OK right! \n");

    Eigen::Vector3d ra; ra << right_arm_q[0][0], right_arm_q[0][1], right_arm_q[0][2];
    Eigen::Vector3d re; re << right_arm_q[1][0], right_arm_q[1][1], right_arm_q[1][2];
    Eigen::Vector3d rw; rw << right_arm_q[2][0], right_arm_q[2][1], right_arm_q[2][2];
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    
    tf.translation() = tf.linear()*ra / 1000.0;
    _world->getSkeleton("rshoulder")->setConfig( dart::math::logMap(Twc*tf) );    
    tf.translation() = tf.linear()*re / 1000.0;
    _world->getSkeleton("relbow")->setConfig( dart::math::logMap(Twc*tf) );
    tf.translation() = tf.linear()*rw / 1000.0;
    _world->getSkeleton("rhand")->setConfig( dart::math::logMap(Twc*tf) );
  }
  
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
  } else if( ACH_OK == r ) {
    printf("Ach is OK upper! \n");
    
    Eigen::Vector3d head; head << upper_body_q[0][0], upper_body_q[0][1], upper_body_q[0][2];
    Eigen::Vector3d neck; neck << upper_body_q[1][0], upper_body_q[1][1], upper_body_q[1][2];
    Eigen::Vector3d torso; torso << upper_body_q[2][0], upper_body_q[2][1], upper_body_q[2][2];
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    
    tf.translation() = tf.linear()*head / 1000.0;
    _world->getSkeleton("head")->setConfig( dart::math::logMap(Twc*tf) );    
    tf.translation() = tf.linear()*neck / 1000.0;
    _world->getSkeleton("neck")->setConfig( dart::math::logMap(Twc*tf) );
    tf.translation() = tf.linear()*torso / 1000.0;
    _world->getSkeleton("torso")->setConfig( dart::math::logMap(Twc*tf) );

  }

  
}



/**
 * @function
 * @brief
 */
void piranha_viz_ui::empty_3_1() {

}

/**
 * @function
 * @brief
 */
void piranha_viz_ui::empty_3_2() {

}

/**
 * @function
 * @brief
 */
void piranha_viz_ui::empty_3_3() {
}





/**
 * @function GRIPEventSceneLoaded
 * @brief Run right after an scene has been loaded
 */
void piranha_viz_ui::GRIPEventSceneLoaded() {

}


void piranha_viz_ui::GRIPEventSimulationBeforeTimestep() {}
void piranha_viz_ui::GRIPEventSimulationAfterTimestep(){}
void piranha_viz_ui::GRIPEventSimulationStart(){}
void piranha_viz_ui::GRIPEventSimulationStop(){}

/**
 * @function GRIPEventTreeViewSelectionChanged
 * @brief Execute when an object is highlighted in the tree
 */
void piranha_viz_ui::GRIPEventTreeViewSelectionChanged() {

    if(!_activeNode) {
        std::cerr << "[piranha_viz_ui] No item selected in TreeView" << std::endl;
        return;
    }

    std::cerr << "[piranha_viz_ui] ActiveNodeType: " << _activeNode->dType << std::endl;
    if(Return_Type_Robot == _activeNode->dType) {
        dart::dynamics::Skeleton* skel = (dart::dynamics::Skeleton*)_activeNode->object;
        std::cerr << "[piranha_viz_ui] Skeleton Selected: " << skel->getName() << std::endl;
    } else if(Return_Type_Node == _activeNode->dType) {
        dart::dynamics::BodyNode* node = (dart::dynamics::BodyNode*)_activeNode->object;
        std::cerr << "[piranha_viz_ui] BodyNode Selected: " << node->getName() << std::endl;
    } else {
        std::cerr << "[piranha_viz_ui] Unknown type selected in TreeView" << std::endl;
    }
}

void piranha_viz_ui::GRIPEventPlaybackBeforeFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed after every playback time step
 */
void piranha_viz_ui::GRIPEventPlaybackAfterFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the start of the playback
 */
void piranha_viz_ui::GRIPEventPlaybackStart() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the end of the playback
 */
void piranha_viz_ui::GRIPEventPlaybackStop() {}

void piranha_viz_ui::Refresh() {}

Q_EXPORT_PLUGIN2(piranha_viz_ui, piranha_viz_ui)
