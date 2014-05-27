/**
 * @file piranha_control.cpp
 * @author Copied literally from pirctrl.c 
 */

#include <sns.h>


#include "piranha_control.h"

/**
 * @function piranha_control
 * @brief Constructor
 */
piranha_control::piranha_control() {

}


/**
 * @function ~piranha_control
 * @brief Destructor
 */
piranha_control::~piranha_control() {

}

/**
 * @function init_setup
 */
bool piranha_control::init_setup() {

    // Set log / error / message variables
    sns_init();

    // Reserve space of piranha struct and set time d
    memset( &mCx, 0, sizeof( mCx ) );
    mCx.dt = 1.0 / 250;

    /*
    // Start
    sns_start();

    // Open channels
    sns_chan_open( &mCx.chan_js, "joystick", NULL );
    sns_chan_open( &mCx.chan_ctrl, "pir-ctrl", NULL );
    sns_chan_open( &mCx.chan_ref_torso, "ref-torso", NULL );
    */


    return true;
}
