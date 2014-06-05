/**
 * @file run_filter.cpp
 * @brief Run first in order to start sending pir_state msgs to be read by control
 */
#include "piranha_filter.h"


/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    // Create piranha_filter instance
    piranha_filter pf;

    // Init it
    pf.init();

    // Run loop of updating / sending
    pf.run();

    return 0;
}
