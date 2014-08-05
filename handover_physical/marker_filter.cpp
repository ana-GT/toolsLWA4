/**
 * @file markerDetection_filter.cpp
 * @brief Read channel from DetectionMarker (KinectAR)
 * @brief and resend a smaller message with marker information for
 * @brief The robot, the table and the object
 * @author A. Huaman Q.
 * @input Channel read from zhaan's marker detection
 * @output Channel with marker info relevant for simulation
 * @output: 1. TABLE, 2. ROBOT 3. OBJECT
 * @date 2014/07/05
 */
#include <amino.h>
#include <ach.h>
#include <sns.h>
#include <unistd.h>

/*****************************************/
ach_channel_t chan_marker_full_state;
ach_channel_t chan_marker_simulation_state;
std::string full_state_name("kinectmarkers");
std::string simulation_state_name("sim-markers");
/*****************************************/


struct marker_t {
    double rot[4]; //x,y,z,w
    double trans[3]; // x,y,z
};

enum markers_id {
    KINECT_TABLE_ID = 21,
    KINECT_ROBOT_ID = 0,
    KINECT_OBJ1_ID = 24,
};

int NUM_MARKERS_SIM = 3;


// Function declarations
void readInfo( sns_wt_tf _in,
	       marker_t &_out );


/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    enum ach_status r;
    double rot[4]; double trans[3];
    sns_wt_tf item;

    sns_init();

    // 1. Open channels.
    r = ach_open( &chan_marker_full_state, full_state_name.c_str(), NULL );

    if( r != ACH_OK ) {
	std::cout << "\t [BANG]"<<full_state_name<<" channel failed to open"<< std::endl;
	return 1;	
    } else {
	std::cout << "\t [GOOD]"<<full_state_name<<" channel opened correctly"<<std::endl;
    }
    r = ach_open( &chan_marker_simulation_state, simulation_state_name.c_str(), NULL );

    if( r != ACH_OK ) {
	std::cout << "\t [BANG]"<<simulation_state_name<<" channel failed to open"<< std::endl;
	return 1;	
    } else {
	std::cout << "\t [GOOD]"<<simulation_state_name<<" channel opened correctly"<<std::endl;
    }



    // Loop
    std::cout << "\t [INFO] Start reading info"<<std::endl;
    while( true ) {

	sns_msg_wt_tf* marker_state_msg = NULL;
	size_t frame_size;
	int n;
	
	marker_state_msg = sns_msg_wt_tf_local_alloc(32);

	// 2. Read channels
	r = ach_get( &chan_marker_full_state,
			       marker_state_msg, 2104,
			       &frame_size,
			       NULL,
			       ACH_O_COPY );

	if( r == ACH_OK ) {
	    std::cout << "\t [DEBUG] ACH WAS OK"<< std::endl;
	} else if( r == ACH_MISSED_FRAME ){
	    std::cout << "\t [DEBUG] ACH MISSED FRAMES"<< std::endl;
	} else if( r == ACH_STALE_FRAMES ) {
	    std::cout <<"\t [DEBUG] ACH STALE FRAMES"<<std::endl;
	} else if( frame_size != sizeof(marker_state_msg) ) {
	    std::cout<<"\t [DEBUG] Unexpected message size: "<< frame_size<<" expecting "<< sizeof( marker_state_msg) << std::endl;	
	} else {
	    std::cout << "\t [CRAP] Something terrible?"<<std::endl;    
	}
	
	n = marker_state_msg->header.n;
	
	// 3. [DEBUG] Show data
	for( int i = 0; i < n; ++i ) {
	    item = marker_state_msg->wt_tf[i];
	    rot[0] = item.tf.r.x;
	    rot[1] = item.tf.r.y;
	    rot[2] = item.tf.r.z;
	    rot[3] = item.tf.r.w;
	    
	    trans[0] = item.tf.v.x;
	    trans[1] = item.tf.v.y;
	    trans[2] = item.tf.v.z;

	    if( rot[0] == 0 && rot[1] == 0 && rot[2] == 0 && rot[3] == 0 && 
		trans[0] == 0 && trans[1] == 0 && trans[2] == 0 ) {
		continue;
	    } else {
		std::cout << "\t * Marker ["<<i<<"] Rot: ("<< rot[0]<<", "<< rot[1]<<","<<rot[2]<<","<<rot[3] <<")";
		std::cout << " and Trans: ("<< trans[0]<< ", "<< trans[1] <<", "<< trans[2]<<") "<< std::endl;
	    }
	} // end for

	/** 4. Send experiment filter information */
	marker_t msg_sim[NUM_MARKERS_SIM];
	// 4.1. TABLE
	item = marker_state_msg->wt_tf[KINECT_TABLE_ID];
	readInfo( item, msg_sim[0] ); 
	// 4.1. ROBOT
	item = marker_state_msg->wt_tf[KINECT_ROBOT_ID];
	readInfo( item, msg_sim[1] ); 
	// 4.1. OBJECT
	item = marker_state_msg->wt_tf[KINECT_OBJ1_ID];
	readInfo( item, msg_sim[2] ); 

	r = ach_put( &chan_marker_simulation_state,
		     msg_sim,
		     sizeof(msg_sim) );
	if( r != ACH_OK ) {
	    std::cout <<"Crap, something went wront with sending simulation info"<< std::endl;
	} else {
	    std::cout << "Things went okay with sim info"<< std::endl;
	}

	// 5. Sleep a tiny bit
	usleep(100000); //0.1 seconds
	    
    } // end while

    //-- Close kinectmarkers channel 
    r = ach_close( &chan_marker_full_state );
    if( r == ACH_OK ) {
	std::cout << "\t [GOOD] Did close markers channel correctly"<< std::endl;
    } else {
	std::cout << "\t [BANG] Did not close markers channel correctly"<< std::endl;
    }

    return 0;
}

/**
 * @function readInfo
 */
void readInfo( sns_wt_tf _in,
	       marker_t &_out ) {

    _out.rot[0] = _in.tf.r.x;
    _out.rot[1] = _in.tf.r.y;
    _out.rot[2] = _in.tf.r.z;
    _out.rot[3] = _in.tf.r.w;

    _out.trans[0] = _in.tf.v.x;
    _out.trans[1] = _in.tf.v.y;
    _out.trans[2] = _in.tf.v.z;

}
