/**
 * @file piranha_control_ui.h
 */
#pragma once

#include <grip/qtWidgets/GripTab.h>
#include <grip/osgGolems/ViewerWidget.h>
#include <grip/qtWidgets/TreeViewReturn.h>

#include "ui_piranha_control_ui.h"

//#include "crichtonSim/crichtonSim.h"
class piranha_control;

namespace dart { namespace dynamics { class Skeleton; } }

/**
 * @class piranha_control_ui
 */
class piranha_control_ui : public GripTab {
    
  Q_OBJECT
    Q_INTERFACES(GripTab)
    
    public:
  piranha_control_ui(QWidget *parent = 0);
  ~piranha_control_ui();
  
  QTimer mTimer;
  
 private:

  piranha_control* mPirCtrl;
   
  std::vector<int> leftIndices;
  Eigen::VectorXd leftValues;
  int counter;


  // Debug tools
  clock_t ts, tf; double dt;
  
  private slots:
  void init_setup();
  void stopAchComm();
  void initUpdate();
  void stopUpdate();
  void initializeSimParams();
  
  void readArmChannels();
  
  void empty_2_1();
  void empty_2_2();
  void empty_2_3();
  void empty_3_1();
  void empty_3_2();
  void empty_3_3();
  
 private:
  void GRIPEventSceneLoaded();
  void GRIPEventSimulationBeforeTimestep();
  void GRIPEventSimulationAfterTimestep();
  void GRIPEventSimulationStart();
  void GRIPEventSimulationStop();
  void GRIPEventPlaybackBeforeFrame();
  void GRIPEventPlaybackAfterFrame();
  void GRIPEventPlaybackStart();
  void GRIPEventPlaybackStop();
  void GRIPEventTreeViewSelectionChanged();
  void Refresh();
  
 private:
  Ui::piranha_control_ui *ui;
  
};
