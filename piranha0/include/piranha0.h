/**
 * @file piranha0.h
 */
#pragma once

#include "ui_piranha0Tab.h"
#include <grip/qtWidgets/GripTab.h>
#include <grip/osgGolems/ViewerWidget.h>
#include <grip/qtWidgets/TreeViewReturn.h>

#include "crichtonSim/crichtonSim.h"


namespace dart { namespace dynamics { class Skeleton; } }

/**
 * @class piranha0
 */
class piranha0 : public GripTab {
    
  Q_OBJECT
    Q_INTERFACES(GripTab)
    
    public:
  piranha0(QWidget *parent = 0);
  ~piranha0();
  
  QTimer mTimer;
  
 private:
  crichtonSim mCrichton;
   
  std::vector<int> leftIndices;
  Eigen::VectorXd leftValues;
  int counter;


  // Debug tools
  clock_t ts, tf; double dt;
  
  private slots:
  void startAchComm();
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
  Ui::piranha0Tab *ui;
  
};
