/**
 * @file piranha0.cpp
 */
 
#include "piranha0.h"
#include <iostream>
#include <qplugin.h>
#include <QtGui>
#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/Joint.h>
#include <dart/dynamics/BodyNode.h>

/**
 * @function piranha0
 * @brief Constructor
 */
piranha0::piranha0(QWidget *parent) : ui(new Ui::piranha0Tab){
    ui->setupUi(this);

    // Set button connections
    connect( ui->startAchComm_pushButton, SIGNAL(released()), 
	     this, SLOT(startAchComm()) );
    connect( ui->stopAchComm_pushButton, SIGNAL(released()), 
	     this, SLOT(stopAchComm()) );
    connect( ui->initUpdate_pushButton, SIGNAL(released()), 
	     this, SLOT(initUpdate()) );
    connect( ui->stopUpdate_pushButton, SIGNAL(released()), 
	     this, SLOT(stopUpdate()) );


    connect( ui->initializeSimParams_pushButton, SIGNAL(released()), 
	     this, SLOT(initializeSimParams()) );

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
    connect( &mTimer, SIGNAL(timeout()), this, SLOT( readArmChannels()));

    // STUFF...
    counter = 0;
    leftIndices.resize(1);
    leftValues.resize(1);
}

/**
 * @function ~piranha0
 * @brief Destructor
 */
piranha0::~piranha0(){

}

/**
 * @function startAchComm
 * @brief
 */
void piranha0::startAchComm() {

}

/**
 * @function
 * @brief
 */
void piranha0::stopAchComm() {

}

/**
 * @function initUpdate()
 * @brief
 */
void piranha0::initUpdate() {
  mTimer.start(100);
}

/**
 * @function stopUpdate()
 * @brief
 */
void piranha0::stopUpdate() {
  mTimer.stop();
}


/**
 * @function
 * @brief
 */
void piranha0::readArmChannels() {

  std::cout << " readArm Channels" << std::endl;
  leftIndices[0] = mCrichton.getArm(LEFT)->getBodyNode("L2_link")->getParentJoint()->getGenCoord(0)->getSkeletonIndex();
     
  // Set joint value
  leftValues[0] = (double)( counter + 1 ) * 2.0 * M_PI / 200.0;

  std::cout << "Left indices: "<< leftIndices[0] << std::endl;
  std::cout << " Left value: "<< leftValues[0] << std::endl;


  mCrichton.getArm(LEFT)->setConfig( leftIndices, leftValues );
  
  std::cout << "Set config" << std::endl;
  counter = ( counter + 1 ) % 200;

}

/**
 * @function initializeSimParams
 * @brief
 */
void piranha0::initializeSimParams() {

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
void piranha0::empty_2_1() {

}

/**
 * @function
 * @brief
 */
void piranha0::empty_2_2() {

}

/**
 * @function
 * @brief
 */
void piranha0::empty_2_3() {

}

/**
 * @function
 * @brief
 */
void piranha0::empty_3_1() {

}

/**
 * @function
 * @brief
 */
void piranha0::empty_3_2() {

}

/**
 * @function
 * @brief
 */
void piranha0::empty_3_3() {
}





/**
 * @function GRIPEventSceneLoaded
 * @brief Run right after an scene has been loaded
 */
void piranha0::GRIPEventSceneLoaded() {
      
    std::cerr << "Test!" << std::endl;

    // Get GolemHubo skeleton
    dart::dynamics::Skeleton* skel = _world->getSkeleton("tetrapak");

    if (skel) {
        // Get index of LSP (left shoulder pitch
        std::vector<int> index(1);
        index[0] = skel->getJoint("LSP")->getGenCoord(0)->getSkeletonIndex();

        // Initialize joint value for LSP
        Eigen::VectorXd jointValue(1);

        // Move joint around
        for (size_t i = 0; i < 200; ++i) {
            // Set joint value
            jointValue[0] = float(i) * 2 * M_PI / 200;
            skel->setConfig(index, jointValue);

            // Save world to timeline
            _timeline->push_back(GripTimeslice(*_world));
        }
    } else {
        std::cerr << "No skeleton named lwa4" << std::endl;
    }


}


void piranha0::GRIPEventSimulationBeforeTimestep() {}
void piranha0::GRIPEventSimulationAfterTimestep(){}
void piranha0::GRIPEventSimulationStart(){}
void piranha0::GRIPEventSimulationStop(){}

/**
 * @function GRIPEventTreeViewSelectionChanged
 * @brief Execute when an object is highlighted in the tree
 */
void piranha0::GRIPEventTreeViewSelectionChanged() {

    if(!_activeNode) {
        std::cerr << "[piranha0] No item selected in TreeView" << std::endl;
        return;
    }

    std::cerr << "[piranha0] ActiveNodeType: " << _activeNode->dType << std::endl;
    if(Return_Type_Robot == _activeNode->dType) {
        dart::dynamics::Skeleton* skel = (dart::dynamics::Skeleton*)_activeNode->object;
        std::cerr << "[piranha0] Skeleton Selected: " << skel->getName() << std::endl;
    } else if(Return_Type_Node == _activeNode->dType) {
        dart::dynamics::BodyNode* node = (dart::dynamics::BodyNode*)_activeNode->object;
        std::cerr << "[piranha0] BodyNode Selected: " << node->getName() << std::endl;
    } else {
        std::cerr << "[piranha0] Unknown type selected in TreeView" << std::endl;
    }
}

void piranha0::GRIPEventPlaybackBeforeFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed after every playback time step
 */
void piranha0::GRIPEventPlaybackAfterFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the start of the playback
 */
void piranha0::GRIPEventPlaybackStart() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the end of the playback
 */
void piranha0::GRIPEventPlaybackStop() {}

void piranha0::Refresh() {}

Q_EXPORT_PLUGIN2(piranha0, piranha0)
