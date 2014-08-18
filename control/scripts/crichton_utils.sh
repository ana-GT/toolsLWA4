#!/bin/sh

#********************************************
# File structure
#********************************************
# 0. Set variables
# 1. Locates $SNS
# 2. Create channels
# 3. Define some useful functions
# 4. Set script options
#********************************************

#************************
# 0. Set variables
#************************

# ARMS + TORSO: SocketCAN interfaces
CAN_L=can0
CAN_R=can1
CAN_T=can2

# HANDS: esd CAN Interfaces
CAN_SDH_L=0
CAN_SDH_R=1


#***********************
# 1. LOCATE $SNS
#***********************

# If $SNS string is empty:
if test -z $SNS; then
    # If sns in this path has executable permissions
    if test -x ~/local/etc/init.d/sns; then
	SNS=~/local/etc/init.d/sns
    elif test -x /usr/local/etc/init.d/sns; then
	SNS=/usr/local/etc/init.d/sns
    elif test -x /etc/init.d/sns; then
	SNS=/etc/init.d/sns
    else
	echo "[ERROR] Could not find SNS program in any of the default locations"
	exit 1
    fi
fi

#*************************
# 2. CREATE CHANNELS
#*************************
CHANNELS="ref-left state-left ref-right state-right"
#CHANNELS="$CHANNELS ref-torso state-torso"
#CHANNELS="$CHANNELS sdhref-left sdhstate-right sdhref-right sdhstate-right"
#CHANNELS="$CHANNELS ft-left ft-right ft-bias-left ft-bias-right"


#******************************
# 3. DEFINE USEFUL FUNCTIONS
#******************************

# Create channels for arm control
crichton_utils_ach_mk() {
    for c in $CHANNELS; do
	ach mk -1 -o 666 $c
    done 
}

# Remove channels for arm control
crichton_utils_ach_rm() {
    for c in $CHANNELS; do
	ach rm $c
    done
}

# Start: Create channels + run daemons
crichton_utils_start() {
    
    # Create channels
    crichton_utils_ach_mk

    # Run daemons can402 for left and right arm
    $SNS run -d -r lwa-left -- \
	can402 -f $CAN_L -R 1 -C 0 -n 3 -n 4 -n 5 -n 6 -n 7 -n 8 -n 9 -c ref-left -s state-left -vvvv
    $SNS run -d -r lwa-right -- \
	can402 -f $CAN_R -R 1 -C 0 -n 3 -n 4 -n 5 -n 6 -n 7 -n 8 -n 9 -c ref-right -s state-right -vvvv

    # Run daemon can402 for torso
    #$SNS run -d torso -- \
    #can402 -f can2 -R 1 -C 0 -n a -c ref-torso -s state-torso

    # Run daemon cftd for Force Torque sensor
    #$SNS run -d -r ft-left -- \
	#cftd -f $CAN_L -n 27 -c ft-left -b ft-bias-left -W 1e6 -F 500
    #$SNS run -d -r ft-right -- \
	#cftd -f $CAN_R -n 27 -c ft-right -b ft-bias-right -W 1e6 -F 500

    # Run sdhiod daemon for SDH hands
    #$SNS run -d -r sdh-left -- \
	#sdhiod -b $CAN_SDH_L -c sdhref-left -s sdh-state-left
    #$SNS run -d -r sdh-right -- \
	#sdhiod -b $CAN_SDH_R -c sdhref-right -s sdhstate-right

}

# Expunge: Remove temporal folders for log
crichton_utils_expunge() {
    sudo rm -rf /var/tmp/sns/lwa-left
    sudo rm -rf /var/tmp/sns/lwa-right
    #sudo rm -rf /var/tmp/sns/torso
    #sudo rm -rf /var/tmp/sns/ft-left
    #sudo rm -rf /var/tmp/sns/ft-right
    #sudo rm -rf /var/tmp/sns/sdh-left
    #sudo rm -rf /var/tmp/sns/sdh-right


    sudo rm -rf /var/run/sns/lwa-left
    sudo rm -rf /var/run/sns/lwa-right
    #sudo rm -rf /var/run/sns/torso
    #sudo rm -rf /var/run/sns/ft-left
    #sudo rm -rf /var/run/sns/ft-right
    #sudo rm -rf /var/run/sns/sdh-left
    #sudo rm -rf /var/run/sns/sdh-right
}

# Stop: Stop daemons and programs
crichton_utils_stop() {
    $SNS kill lwa-left
    $SNS kill lwa-right
    #$SNS kill torso
    #$SNS kill ft-left
    #$SNS kill ft-right
    #$SNS kill sdh-left
    #$SNS kill sdh-right
}

crichton_utils_steal() {
    chown -R $1 /var/run/sns/lwa-left \
	/var/run/sns/lwa-right \
	#/var/run/sns/torso \
	#/var/run/sns/ft-left \
	#/var/run/sns/ft-right \
	#/var/run/sns/sdh-left \
	#/var/run/sns/sdh-right \
	/var/tmp/sns/lwa-left \
	/var/tmp/sns/lwa-right #\
	#/var/tmp/sns/torso \
	#/var/tmp/sns/ft-left \
	#/var/tmp/sns/ft-right \
	#/var/tmp/sns/sdh-left \
	#/var/tmp/sns/sdh-right 
}

#*************************
# 4. Set script options
#**************************

case "$1" in
    start)
	crichton_utils_start
	;;
    stop)
	crichton_utils_stop
	;;
    rm)
	crichton_utils_ach_rm
	;;
    mk) cricthon_utils_ach_mk
	;;
    steal)
	shift
	crichton_utils_steal $@
	;;
    expunge)
	crichton_utils_expunge
	;;
    *)
	echo "[ERROR] Invalid command. Options are start / stop / rm / mk / steal NEW_OWNER / expunge"
	exit 1
	;;
esac
