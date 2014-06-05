/**
 * @function piranha_control.h
 * @author Copied from pirctrl (package piranha)
 */
#pragma once

#include <amino.h>
#include <ach.h>
#include <reflex.h>
#include <piranha.h>

/**
 * @class piranha_control
 */
class piranha_control {

 public:

    piranha_control();
    ~piranha_control();

    bool init_setup();

 private:
    pirctrl_cx_t mCx;

};
