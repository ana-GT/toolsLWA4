/**
 * @file subjectTracking.cpp
 */

#include "subjectTracking.h"


/**
 *
 */
int main( int argc, char* argv[] ) {


  subjectTracking st;
  st.init();

  
  while( !xnOSWasKeyboardHit() ) {
    st.update();
    //usleep(250*1000);
  } 


  printf("Finishing gracefully! Calling cleanup and saying good bye \n");
  st.CleanupExit();



  return 0;
}
