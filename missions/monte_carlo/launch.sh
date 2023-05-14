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

PLANNER="lpastar"; PLANNER_OPTIONS=($PLANNER "dstar_lite" "gcs" "gcs_r")
NUM_TRIALS=3

USE_OBS_AVOID="true"
DRIFT_STRENGTH=0
DRIFT_DIR="x"; DRIFT_DIR_OPTIONS=($DRIFT_DIR "y" "random")

RANDOM_OBS_REGION="-135,-200:-90,-50:45,17:250,17:250,-50:185,-200"
RANDOM_OBS_MIN_RANGE=10
RANDOM_OBS_MIN_SIZE=8
RANDOM_OBS_MAX_SIZE=15
RANDOM_OBS_AMT=7


#----------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#----------------------------------------------------------
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
        echo "    Planner to run, must be one of [${PLANNER_OPTIONS[@]}]."
        echo "    Default is \"$PLANNER\".                       "
        echo "  --num_trials=<num>, -n=<num>                     "
        echo "    Number of Monte Carlo trials to run, must be an integer."
        echo "  --no_obs_avoid                                   "
        echo "    Do not use pObstacleMgr obstacle avoidance behaviors during trials."
        echo "  --drift_strength=<strength>                      "
        echo "    Strength of current drifts disturbing vehicle motion. Default is "
        echo "    0 for no drifts.                               "
        echo "  --drift_dir=<dir>                                "
        echo "    Direction of drifts disturbing vehicle motion, must be one of "
        echo "    [${DRIFT_DIR_OPTIONS[@]}]. Default is \"$DRIFT_DIR\"."
        echo "    If random, drifts are produced with a random direction "
        echo "    and a magnitude between 0.5 and 2."
        exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ] ; then
        JUST_MAKE="true"
    elif [ "${ARGI}" = "--nogui" ] ; then
        GUI="false"

    elif [ "${ARGI::10}" = "--planner=" -o "${ARGI::3}" = "-p=" ] ; then
        PLANNER=${ARGI#*=}  # remove arg name by splitting at "="
        PLANNER=$(echo $PLANNER | sed "s/[A-Z]/\L&/g")  # make arg lowercase
        if [[ ! " ${PLANNER_OPTIONS[*]} " =~ " ${PLANNER} " ]]; then
            echo "launch.sh Bad arg: $ARGI"
            echo "Value must be one of [${PLANNER_OPTIONS[@]}]"
            echo "Exiting with code: 1"
            exit 1;
        fi
    elif [ "${ARGI::13}" = "--num_trials=" -o "${ARGI::3}" = "-n=" ] ; then
        NUM_TRIALS="${ARGI#*=}"

    elif [ "${ARGI}" = "--no_obs_avoid" ] ; then
        USE_OBS_AVOID="false"
    elif [ "${ARGI::17}" = "--drift_strength=" ] ; then
        DRIFT_STRENGTH="${ARGI#*=}"
    elif [ "${ARGI::12}" = "--drift_dir=" ] ; then
        DRIFT_DIR=${ARGI#*=}  # remove arg name by splitting at "="
        DRIFT_DIR=$(echo $DRIFT_DIR | sed "s/[A-Z]/\L&/g")  # make arg lowercase
        if [[ ! " ${DRIFT_DIR_OPTIONS[*]} " =~ " ${DRIFT_DIR} " ]]; then
            echo "launch.sh Bad arg: $ARGI"
            echo "Value must be one of [${DRIFT_DIR_OPTIONS[@]}]"
            echo "Exiting with code: 1"
            exit 1;
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

PATH_REQUEST_VAR="PLAN_PATH_REQUESTED"
PATH_FOUND_VAR="PATH_FOUND"
PATH_COMPLETE_VAR="PATH_COMPLETE"

# V1 configuration
V1_NAME="artemis"
V1_START_X="-100"; V1_START_Y="-220"
V1_START_POS="$V1_START_X,$V1_START_Y"
V1_START_SPEC="x=$V1_START_X,y=$V1_START_Y"
V1_GOAL_X="210"; V1_GOAL_Y="30"
V1_GOAL_POS="$V1_GOAL_X,$V1_GOAL_Y"
V1_GOAL_SPEC="x=$V1_GOAL_X,y=$V1_GOAL_Y"
V1_COLOR="red"
V1_MOOSDB="9100"
V1_PSHARE="9300"


# generate obstacle files
nsplug meta_obstacles_const.txt $OBS_CONST_FILE -i -f
gen_obstacles --poly=$RANDOM_OBS_REGION  --min_range=$RANDOM_OBS_MIN_RANGE    \
              --max_size=$RANDOM_OBS_MAX_SIZE --min_size=$RANDOM_OBS_MIN_SIZE \
              --amt=$RANDOM_OBS_AMT > $OBS_KNOWN_FILE
sleep 1  # sleep for a bit so gen_obstacles gets a new random seed (based on sys time)
gen_obstacles --poly=$RANDOM_OBS_REGION  --min_range=$RANDOM_OBS_MIN_RANGE    \
              --max_size=$RANDOM_OBS_MAX_SIZE --min_size=$RANDOM_OBS_MIN_SIZE \
              --amt=$RANDOM_OBS_AMT > $OBS_UNKNOWN_FILE


nsplug meta_shoreside.moos targ_shoreside.moos -i -f WARP=$TIME_WARP \
       IP_ADDR="localhost"    SHORE_MOOSDB=$SHORE_MOOSDB             \
       PSHARE_PORT=$SHORE_PSHARE    V1_NAME=$V1_NAME    GUI=$GUI      \
       NUM_TRIALS=$NUM_TRIALS    OBS_CONST_FILE=$OBS_CONST_FILE      \
       OBS_KNOWN_FILE=$OBS_KNOWN_FILE    OBS_UNKNOWN_FILE=$OBS_UNKNOWN_FILE\
       START_POS=$V1_START_SPEC    GOAL_POS=$V1_GOAL_SPEC            \
       PATH_REQUEST_VAR=$PATH_REQUEST_VAR    PATH_COMPLETE_VAR=$PATH_COMPLETE_VAR

# todo: vehicle start/goal positions should be sent from uEvalPlanner
nsplug meta_vehicle.moos targ_$V1_NAME.moos -i -f WARP=$TIME_WARP  \
       IP_ADDR="localhost"    VNAME=$V1_NAME                       \
       V_MOOSDB=$V1_MOOSDB    PSHARE_PORT=$V1_PSHARE               \
       SHORE_IP="localhost"    SHORE_PSHARE=$SHORE_PSHARE          \
       START_POS=$V1_START_POS    GOAL_POS=$V1_GOAL_POS            \
       VCOLOR=$V1_COLOR    PLANNER=$PLANNER    DRIFT_DIR=$DRIFT_DIR\
       DRIFT_STRENGTH=$DRIFT_STRENGTH                              \
       PATH_REQUEST_VAR=$PATH_REQUEST_VAR    PATH_FOUND_VAR=$PATH_FOUND_VAR\
       PATH_COMPLETE_VAR=$PATH_COMPLETE_VAR

nsplug meta_vehicle.bhv targ_$V1_NAME.bhv -i -f VNAME=$V1_NAME    \
       START_POS=$V1_START_POS    USE_OBS_AVOID=$USE_OBS_AVOID    \
       PATH_FOUND_VAR=$PATH_FOUND_VAR    PATH_COMPLETE_VAR=$PATH_COMPLETE_VAR


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


if [ $GUI = "false" ] ; then
    sleep 5  # give everything time to boot up
    uPokeDB targ_shoreside.moos DEPLOY_ALL=true MOOS_MANUAL_OVERRIDE_ALL=false RESET_SIM_REQUESTED=all
fi

uMAC -t targ_shoreside.moos
kill -- -$$

if [ $GUI = "false" ] ; then
    sleep 5
    ktm  # kill any stragglers, seems to happen without the gui
fi
