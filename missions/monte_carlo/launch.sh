#!/bin/bash 
#----------------------------------------------------------
#  Script: launch.sh
#  Author: Raul Largaespada
#  LastEd: Apr 13th 2023
#----------------------------------------------------------
#  Part 1: Set global var defaults
#----------------------------------------------------------
TIME_WARP=1
JUST_MAKE="false"
GUI="true"

PLANNER="lpastar"; _PLANNER_OPTIONS=($PLANNER "dstar_lite" "gcs" "gcs_r")
LAYOUT="base"; _LAYOUT_OPTIONS=($LAYOUT "uturn" "sturn")
NUM_TRIALS=100

USE_OBS_AVOID="true"
DRIFT_STRENGTH=0
DRIFT_DIR="x"; _DRIFT_DIR_OPTIONS=($DRIFT_DIR "y" "random")


#----------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#----------------------------------------------------------
# make sure argument is valid 
function validate_arg () {
    local user_arg=$1  # save first arg to a variable
    shift  # shift all args to the left (original $1 is lost)
    local valid_args=("$@")  # build array with remaining args

    local arg_value=${user_arg#*=}  # remove arg name by splitting at "="
    arg_value=$(echo "$arg_value" | awk '{print tolower($0)}')  # make arg lowercase

    # if arg is not valid, exit with code 1
    if [[ ! " ${valid_args[*]} " =~ " ${arg_value} " ]]; then
        echo "launch.sh Bad arg: \"$user_arg\";"
        echo "Value must be one of [${valid_args[@]}];"
        echo "Exiting with code: 1"
        return 1;
    fi

    # arg is valid, return arg to script
    echo "$arg_value"
    return 0
}


for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
        echo "launch.sh [SWITCHES] [time_warp]                   "
        echo "  --help, -h                                       "
        echo "    Show this help message.                        "
        echo "  --just_make, -j                                  "
        echo "    Just create targ files, no launch.             "
        echo "  --nogui                                          "
    	echo "    Do not launch pMarineViewer GUI with vehicle.  "
        echo "  --planner=<planner>, -p=<planner>                "
        echo "    Planner to run, must be one of [${_PLANNER_OPTIONS[@]}]."
        echo "    Default is \"$PLANNER\".                       "
        echo "  --layout=<layout>, -l=<layout>                   "
        echo "    Layout of constant obstacles, must be one of [${_LAYOUT_OPTIONS[@]}]."
        echo "    Default is \"$LAYOUT\".                        "
        echo "  --num_trials=<num>, -n=<num>                     "
        echo "    Number of Monte Carlo trials to run, must be an integer."
        echo "  --no_obs_avoid                                   "
        echo "    Do not use pObstacleMgr obstacle avoidance behaviors during trials."
        echo "  --drift_strength=<strength>                      "
        echo "    Strength of current drifts disturbing vehicle motion. Default is "
        echo "    0 for no drifts.                               "
        echo "  --drift_dir=<dir>                                "
        echo "    Direction of drifts disturbing vehicle motion, must be one of "
        echo "    [${_DRIFT_DIR_OPTIONS[@]}]. Default is \"$DRIFT_DIR\"."
        exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ] ; then
        JUST_MAKE="true"
    elif [ "${ARGI}" = "--nogui" ] ; then
        GUI="false"

    elif [ "${ARGI::10}" = "--planner=" -o "${ARGI::3}" = "-p=" ] ; then
        PLANNER="$(validate_arg $ARGI ${_PLANNER_OPTIONS[@]})"
        if [ $? = 1 ] ; then  # check return code of validate_arg
            echo $PLANNER
            exit 1
        fi
    elif [ "${ARGI::9}" = "--layout=" -o "${ARGI::3}" = "-l=" ] ; then
        LAYOUT="$(validate_arg $ARGI ${_LAYOUT_OPTIONS[@]})"
        if [ $? = 1 ] ; then  # check return code of validate_arg
            echo $LAYOUT
            exit 1
        fi
    elif [ "${ARGI::13}" = "--num_trials=" -o "${ARGI::3}" = "-n=" ] ; then
        NUM_TRIALS="${ARGI#*=}"

    elif [ "${ARGI}" = "--no_obs_avoid" ] ; then
        USE_OBS_AVOID="false"
    elif [ "${ARGI::17}" = "--drift_strength=" ] ; then
        DRIFT_STRENGTH="${ARGI#*=}"
    elif [ "${ARGI::12}" = "--drift_dir=" ] ; then
        DRIFT_DIR="$(validate_arg $ARGI ${_DRIFT_DIR_OPTIONS[@]})"
        if [ $? = 1 ] ; then  # check return code of validate_arg
            echo $DRIFT_DIR
            exit 1
        fi
    else 
        echo "launch.sh Bad arg:" $ARGI " Exiting with code: 1"
        exit 1
    fi
done


#-------------------------------------------------------
#  Part 3: Create the .moos and .bhv files. 
#-------------------------------------------------------
# Shoreside config
SHORE_MOOSDB="9000"
SHORE_PSHARE="9200"
OBS_CONST_FILE="targ_obstacles_const.txt"
OBS_KNOWN_FILE="targ_obstacles_known.txt"
OBS_UNKNOWN_FILE="targ_obstacles_unknown.txt"

# define obstacle region, pMarineViewer pan based on layout


# V1 configuration
V1_NAME="artemis"
# V1_START_POS="-100,-220"  # depends on LAYOUT
# V1_GOAL_POS="210,30"  # depends on LAYOUT
V1_START_POS="0,-20"  # depends on LAYOUT
V1_GOAL_POS="0,0"  # depends on LAYOUT
V1_COLOR="red"
V1_MOOSDB="9100"
V1_PSHARE="9300"


# generate obstacle files
nsplug meta_obstacles_const.txt $OBS_CONST_FILE -i -f LAYOUT=$LAYOUT


nsplug meta_shoreside.moos targ_shoreside.moos -i -f WARP=$TIME_WARP \
       IP_ADDR="localhost"    SHORE_MOOSDB=$SHORE_MOOSDB             \
       PSHARE_PORT=$SHORE_PSHARE  LAYOUT=$LAYOUT    VNAMES=$V1_NAME  \
       NUM_TRIALS=$NUM_TRIALS    GUI=$GUI                            \
       OBS_CONST_FILE=$OBS_CONST_FILE

nsplug meta_vehicle.moos targ_$V1_NAME.moos -i -f WARP=$TIME_WARP  \
       IP_ADDR="localhost"    VNAME=$V1_NAME                       \
       V_MOOSDB=$V1_MOOSDB    PSHARE_PORT=$V1_PSHARE               \
       SHORE_IP="localhost"    SHORE_PSHARE=$SHORE_PSHARE          \
       START_POS=$V1_START_POS    GOAL_POS=$V1_GOAL_POS            \
       VCOLOR=$V1_COLOR
# add PLANNER, DRIFT_STRENGTH, DRIFT_DIR

nsplug meta_vehicle.bhv targ_$V1_NAME.bhv -i -f VNAME=$V1_NAME    \
       START_POS=$V1_START_POS    GOAL_POS=$V1_GOAL_POS           \
       USE_OBS_AVOID=$USE_OBS_AVOID


if [ ${JUST_MAKE} = "true" ] ; then
    echo "Files assembled; nothing launched; exiting per request."
    exit 0
fi


#----------------------------------------------------------
#  Part 4: Launch the processes
#----------------------------------------------------------
echo "Launching Shoreside MOOS Community. WARP is" $TIME_WARP
pAntler targ_shoreside.moos >& /dev/null &

echo "Launching $V1_NAME MOOS Community. WARP is" $TIME_WARP
pAntler targ_$V1_NAME.moos >& /dev/null &

uMAC -t targ_shoreside.moos
kill -- -$$
