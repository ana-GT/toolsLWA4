/**
 * @file graspVizApp.h
 * @brief Tests aoi Reach Interaction
 * @author A. Huaman Q.
 */
#include "GRIPApp.h"
#include "graspViz.h"

extern wxNotebook* tabView;

/**
 * @class graspVizApp
 * @brief Test pick, reach and place operations
 */
class graspVizApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new graspViz(tabView), 
				 wxT("Grasp Viz"));
	}
};

IMPLEMENT_APP(graspVizApp)
