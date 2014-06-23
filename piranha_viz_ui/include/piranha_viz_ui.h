/**
 * @file piranha_viz_ui.h
 */
#pragma once

#include <grip/qtWidgets/GripTab.h>
#include <grip/osgGolems/ViewerWidget.h>
#include <grip/qtWidgets/TreeViewReturn.h>

#include "ui_piranha_viz_ui.h"

#include "crichtonSim/crichtonSim.h"

namespace dart { namespace dynamics { class Skeleton; } }

/**
 * @class piranha_viz_ui
 */
class piranha_viz_ui : public GripTab {
    
  Q_OBJECT
    Q_INTERFACES(GripTab)
    
    public:
    piranha_viz_ui(QWidget *parent = 0);
    ~piranha_viz_ui();
  
  QTimer mTimer;
  QTimer mTimer_subject;
  
 private:

  crichtonSim mCrichton;
   

  // Debug tools
  clock_t ts, tf; double dt;
  
  private slots:
  void init_ACH();
  void stop_ACH();
  void startUpdate();
  void stopUpdate();
  void initializeSimParams();
  
  void update();
  
  void init_ACH_subject();
  void init_stuff_subject();
  void startUpdate_subject();
  void stopUpdate_subject();
  void update_subject();


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
  Ui::piranha_viz_ui *ui;
  
  // Subject stuff
  ach_channel_t left_arm_chan;
  ach_channel_t right_arm_chan;
  ach_channel_t upper_body_chan;
  
  double left_arm_q[3][3];
  double right_arm_q[3][3];
  double upper_body_q[3][3];


  Eigen::Isometry3d Twc; /** Transformation from world to camera */

 public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
