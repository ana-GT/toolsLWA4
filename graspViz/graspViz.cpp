/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "graspViz.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <iostream>

#include <Tabs/AllTabs.h>
#include <GRIPApp.h>


// **********************

/** Events */
enum graspVizEvents {
    id_button_selectMesh = 8345,
    id_button_getMeshRep,
    id_button_view_pc_object,
    id_button_view_pc_free,
    id_button_calculate_clusters,
    id_button_view_cluster,
    id_button_collision
};

/** Handler for events **/
BEGIN_EVENT_TABLE( graspViz, wxPanel )
EVT_COMMAND ( wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, graspViz::OnButton )
EVT_COMMAND ( wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, graspViz::OnSlider )
END_EVENT_TABLE()


// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS( graspViz, GRIPTab )


/**
 * @function graspViz
 * @brief Constructor
 */
graspViz::graspViz( wxWindow *parent, const wxWindowID id,
		  const wxPoint& pos, const wxSize& size, long style ) :
GRIPTab( parent, id, pos, size, style ) {
    sizerFull = new wxBoxSizer(wxHORIZONTAL);
    
    // Create Static boxes (outline of your Tab)
    wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Select Mesh"));
    wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Clusters"));
    wxStaticBox* ss3Box = new wxStaticBox(this, -1, wxT("Utils"));
    
    // Create sizers for these static boxes
    wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
    wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);
    wxStaticBoxSizer* ss3BoxS = new wxStaticBoxSizer(ss3Box, wxVERTICAL);
    

    // SS1BoxS
    ss1BoxS->Add( new wxButton( this, id_button_selectMesh, 
				wxT("Select Mesh")), 0, wxALL, 1 );
    ss1BoxS->Add( new wxButton( this, id_button_getMeshRep, 
				wxT("Get Mesh rep")), 0, wxALL, 1 );
    ss1BoxS->Add( new wxButton( this, id_button_view_pc_object, 
				wxT("View object rep")), 0, wxALL, 1 );
    ss1BoxS->Add( new wxButton( this, id_button_view_pc_free, 
				wxT("View free rep")), 0, wxALL, 1 );

    // SS2BoxS
    ss2BoxS->Add( new wxButton( this, id_button_calculate_clusters, 
				wxT("Calculate clusters")), 0, wxALL, 1 );
    ss2BoxS->Add( new wxButton( this, id_button_view_cluster, 
				wxT("View clusters")), 0, wxALL, 1 );

    // SS3BoxS
    ss3BoxS->Add( new wxButton( this, id_button_collision, 
				wxT("Check collision")), 0, wxALL, 1 );

        
    // Add the boxes to their respective sizers
    sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
    sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
    sizerFull->Add(ss3BoxS, 1, wxEXPAND | wxALL, 6);
    
    SetSizer(sizerFull);
    
    // Additional settings
    init();
}

/**< @function init */
void graspViz::init() {
    clusterInd = 0;

    // Set background to white
    viewer->backColor = Vector3d(1,1,1);
    viewer->gridColor = Vector3d(.8,.8,1);
    viewer->setClearColor();
    viewer->DrawGLScene();
}



/**
 * @function GRIPEventSceneLoaded
 * @brief Functions that is called right after the world scene is loaded
 */
void graspViz::GRIPEventSceneLoaded() {
}

/**
 * @function OnButton
 * @brief Handles button events
 */
void graspViz::OnButton(wxCommandEvent & _evt) {
  
    int slnum = _evt.GetId();
    
    switch( slnum ) {
	
	// Select mesh
    case id_button_selectMesh : {
	mObject =  mSelected;
	mu.setObject( mObject );
	std::cout << "Set object : "<< mObject->getName()<< std::endl;
    } break;

	// Get mesh representation (normals and points)
    case id_button_getMeshRep : {
	mu.calculate_rep();
      std::cout << "Got mesh representation "<< std::endl;
    } break;

	// View pc object
    case id_button_view_pc_object : {
	mu.vizObj_rep();
 
    } break;


	// View pc free
    case id_button_view_pc_free : {


    } break;

	// Calculate clusters
    case id_button_calculate_clusters : {
 
    } break;

	
	// View clusters
    case id_button_view_cluster : {
 
    } break;

	
	// Collision
    case id_button_collision : {
	if( !mWorld->checkCollision() ) {
	    std::cout << "No collision detected" << std::endl;
	} else {
	    std::cout << "Oh crab, collision detected" << std::endl;
	}
    } break;


	
    }
}

/**
 * @function OnSlider
 * @brief Handles slider changes
 */
void graspViz::OnSlider( wxCommandEvent &_evt ) {

  int slnum = _evt.GetId();
  
  switch( slnum ) {

  }
    
}


/**
 * @function GRIPEventSimulationBeforeTimeStep
 * @brief Before each sim step we must set the internal forces 
 */
void graspViz::GRIPEventSimulationBeforeTimestep() {
}

/**
 * @function GRIPEventSimulationAfterTimeStep
 * @brief
 */
void graspViz::GRIPEventSimulationAfterTimestep() {
}

/**
 * @function GRIPEventSimulationStart
 * @brief
 */
void graspViz::GRIPEventSimulationStart() {

}



// This function is called when an object is selected in the Tree View or other
// global changes to the GRIP world. Use this to capture events from outside the tab.
void graspViz::GRIPStateChange() {
  if(selectedTreeNode==NULL){
    return;
  }
  
  string statusBuf;
  string buf, buf2;
  switch (selectedTreeNode->dType) {
  case Return_Type_Robot:
    mSelected = (dart::dynamics::Skeleton*) selectedTreeNode->data;
    break;
  case Return_Type_Node:
    // Not much
    break;
  default:
    fprintf(stderr, "someone else's problem.");
    assert(0);
    exit(1);
  }
  //frame->SetStatusText(wxString(statusBuf.c_str(), wxConvUTF8));
  //sizerFull->Layout();
}


