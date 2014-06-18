#!/bin/sh

CHANNELS="subject_larm_state subject_rarm_state subject_upper_state"


# -------------------------------
# tracking_ach_mk
# Create ach channels 
# -------------------------------
tracking_ach_mk() {

    for c in $CHANNELS; do
	ach mk -1 -o 666 $c
    done
} 

# -------------------------------
# tracking_ach_rm
# Remove ach channels created 
# -------------------------------
tracking_ach_rm() {
    for c in $CHANNELS; do
	ach rm $c
    done
}

# ---------------
# tracking_start
# ---------------
tracking_start() {
    tracking_ach_mk
}

# -----------------
# tracking_stop
# -----------------
tracking_stop() {
    echo "I should kill here with SNS kill"
}

# ************************************
# Call functions on input argument
# ************************************
case "$1" in
    start)
	tracking_start
	;;
    stop)
	tracking_stop
	;;
    rm)
	tracking_ach_rm
	;;
    mk)
	tracking_ach_mk
	;;
    *)
	echo "[ERROR] Invalid command"
	exit 1
	;;
esac