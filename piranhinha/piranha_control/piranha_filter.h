/**
 * @function piranha_filter.h
 * @author Copied from pirctrl (package piranha)
 * @brief Reads the channels from the arm/hand/FT and outputs
 * a channel with a channel combining the info in pir_cx_t  
 */
#pragma once

#include <amino.h>
#include <ach.h>
#include <reflex.h>
#include <piranha.h>

/**
 *
 */
typedef struct {

    /** Channels to read / send info */
    ach_channel_t chan_state_torso;
    ach_channel_t chan_state_left;
    ach_channel_t chan_state_right;
    ach_channel_t chan_ft_left;
    ach_channel_t chan_ft_right;
    ach_channel_t chan_ftbias[2];
    ach_channel_t chan_sdhstate_left;
    ach_channel_t chan_sdhstate_right;

    ach_channel_t chan_state_pir;
    ach_channel_t chan_config;

    /** F/T sensor stuff */
    double F_raw[2][6];
    double r_ft_rel[4];
    double S_eer[2][8];
    double r_ft[2][4];

    /** Information to be queried */
    struct pir_config Q;
    struct pir_state state;
    struct timespec now;

    double S0[2][8];
    
    sig_atomic_t rebias;

} filter_data_t; 

/**
 * @class piranha_filter
 */
class piranha_filter {

 public:

    piranha_filter();
    ~piranha_filter();

    bool init();
    void run();

    void update(void);
    int update_n( size_t _n,
		  size_t _i,
		  ach_channel_t *_chan,
		  struct timespec *_ts );
    int update_ft( double *F,
		   ach_channel_t *chan,
		   struct timespec *ts );

    void sighandler_hup( int _sig );
    int bias_ft( void );

 private:
    pirctrl_cx_t mCx;

    filter_data_t mFd;

};
