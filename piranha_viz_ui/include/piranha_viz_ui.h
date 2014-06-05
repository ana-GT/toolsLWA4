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
  Ui::piranha_viz_ui *ui;
  
};
