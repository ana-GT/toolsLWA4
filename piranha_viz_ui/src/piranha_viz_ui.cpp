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


    connect( ui->empty_2_1_pushButton, SIGNAL(released()), 
	     this, SLOT(empty_2_1()) );
    connect( ui->empty_2_2_pushButton, SIGNAL(released()), 
	     this, SLOT(empty_2_2()) );
    connect( ui->empty_2_3_pushButton, SIGNAL(released()), 
	     this, SLOT(empty_2_3()) );

    connect( ui->empty_3_1_pushButton, SIGNAL(released()), 
	     this, SLOT(empty_3_1()) );
    connect( ui->empty_3_2_pushButton, SIGNAL(released()), 
	     this, SLOT(empty_3_2()) );
    connect( ui->empty_3_3_pushButton, SIGNAL(released()), 
	     this, SLOT(empty_3_3()) );

    // Set timer connection
    connect( &mTimer, SIGNAL(timeout()), this, SLOT( update()));

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
  mTimer.start(100);
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

    mCrichton.getStatesToSim();
    

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
 * @function
 * @brief
 */
void piranha_viz_ui::empty_2_1() {

}

/**
 * @function
 * @brief
 */
void piranha_viz_ui::empty_2_2() {

}

/**
 * @function
 * @brief
 */
void piranha_viz_ui::empty_2_3() {

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
