#!/bin/bash -e
#----------------------------------------------------------
#  Script: run_benchmark.sh
#  Author: Raul Largaespada
#  LastEd: June 13th 2023
#
#^ NOTE: This script cannot be run directly, but must be
#^ called by a launch script from a child directory. It is
#^ placed at this level to avoid unnecessary duplication,
#^ since this script is the same for each scenario.
#----------------------------------------------------------
#  Part 1: Pull global var defaults from config files
#----------------------------------------------------------
# define upper level param directories
LAYOUT_DIR=$(dirname $PWD)  # layout directory is one level up
MAP_DIR=$(dirname $LAYOUT_DIR)  # map directory is two levels up
MISSIONS_DIR=$(dirname $MAP_DIR)  # missions directory is three levels up

# source config vars from upper level dirs
source ${MISSIONS_DIR}/config.sh
source ${MAP_DIR}/config.sh
source ${LAYOUT_DIR}/config.sh

# variable overwrites
USE_BENCHMARK="true"
OBS_KNOWN_FILE="tmp/benchmark_obstacles_known"  # suffix and extension are included in .moos file
OBS_UNKNOWN_FILE="tmp/benchmark_obstacles_unknown"  # suffix and extension are included in .moos file


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
        echo "  --export=<filestem>, -e=<filestem>               "
        echo "    CSV file to export simulation results to, without"
        echo "    the \".csv\" suffix. Defaults to \"$EXPORT_FILE\"."
        echo "    File is placed in the \"./${METRICS_DIR}/<planner>/\" directory."
        echo "  --no_obs_avoid                                   "
        echo "    Do not use pObstacleMgr obstacle avoidance behaviors during trials."
        echo "  --drift_vector=<heading>,<magnitude>             "
        echo "    Vector defining water currents affecting obstacle "
        echo "    and vehicle motion. Default is no drifts.      "
        echo "  --wind_vector=<heading>,<magnitude>              "
        echo "    Vector defining winds affecting vehicle motion only. "
        echo "    Default is no winds.                           "
        echo "  --random_gusts                                   "
        echo "    Add random gusts of wind disturbing vehicle motion."
        echo "    Gusts are produced with a random direction and a"
        echo "    random magnitude between 0.5 and 2."
        exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ] ; then
        JUST_MAKE="true"
    elif [ "${ARGI}" = "--nogui" ] ; then
        GUI="false"

    elif [ "${ARGI::10}" = "--planner=" -o "${ARGI::3}" = "-p=" ] ; then
        PLANNER=${ARGI#*=}  # remove arg name by splitting at "="
        PLANNER=${PLANNER,,}  # make arg lowercase
        if [[ ! " ${PLANNER_OPTIONS[*]} " =~ " ${PLANNER} " ]]; then
            echo "launch.sh Bad arg: $ARGI"
            echo "Value must be one of [${PLANNER_OPTIONS[@]}]"
            echo "Exiting with code: 1"
            exit 1;
        fi
    elif [ "${ARGI::9}" = "--export=" -o "${ARGI::3}" = "-e=" ] ; then
        EXPORT_FILE="${ARGI#*=}"
        EXPORT_FILE=${EXPORT_FILE%.*}  # remove file extension if it exists

    elif [ "${ARGI}" = "--no_obs_avoid" ] ; then
        USE_OBS_AVOID="false"
    elif [ "${ARGI::15}" = "--drift_vector=" ] ; then
        DRIFT_VECTOR="${ARGI#*=}"
    elif [ "${ARGI::14}" = "--wind_vector=" ] ; then
        WIND_VECTOR="${ARGI#*=}"
    elif [ "${ARGI}" = "--random_gusts" ] ; then
        RANDOM_GUSTS="true"
    else 
        echo "launch.sh Bad arg:" $ARGI " Exiting with code: 1"
        exit 1
    fi
done

# add parent dirs to export file
EXPORT_FILE=${METRICS_DIR}/${PLANNER}/${EXPORT_FILE}

#-------------------------------------------------------
#  Part 3: Create the .moos and .bhv files
#-------------------------------------------------------

# unzip out benchmark files and save total number
unzip -q benchmark_obstacles.zip
NUM_TRIALS=$(ls tmp | wc -l)  # count number of files unzipped
NUM_TRIALS=$(($NUM_TRIALS/2))  # divide by 2 since files are in known/unknown pairs

# generate constant obstacle file
nsplug "${MAP_DIR}/${OBS_MAP_FILE}" "${OBS_CONST_FILE}" -i -f \
       --path="${LAYOUT_DIR}:${MAP_DIR}:${MISSIONS_DIR}" \
       MAP_BOUNDS="${MAP_BOUNDS}" \
       LAYOUT_OBSTACLES_FILE="${OBS_LAYOUT_FILE}"


# genrate moos files
nsplug ${MISSIONS_DIR}/meta_shoreside.moos targ_shoreside.moos -i -f \
       --path="${LAYOUT_DIR}:${MAP_DIR}:${MISSIONS_DIR}" \
       WARP=$TIME_WARP \
       LAT_ORIGIN=$LAT_ORIGIN \
       LONG_ORIGIN=$LONG_ORIGIN \
       V1_NAME=$V1_NAME \
       IP_ADDR="localhost" \
       SHORE_MOOSDB=$SHORE_MOOSDB \
       PSHARE_PORT=$SHORE_PSHARE \
       GUI=$GUI \
       TIFF_FILE=$TIFF_FILE \
       PMV_PAN_X=$PMV_PAN_X \
       PMV_PAN_Y=$PMV_PAN_Y \
       PMV_ZOOM=$PMV_ZOOM \
       OBS_CONST_FILE="${OBS_CONST_FILE}" \
       OBS_KNOWN_FILE="${OBS_KNOWN_FILE}" \
       OBS_UNKNOWN_FILE="${OBS_UNKNOWN_FILE}" \
       NUM_TRIALS=$NUM_TRIALS \
       START_POS="${V1_START_POS}" \
       GOAL_POS="${V1_GOAL_POS}" \
       HDG_ON_RESET=$HDG_ON_RESET \
       EXPORT_FILE="${EXPORT_FILE}" \
       PLANNER=$PLANNER \
       USE_BENCHMARK=$USE_BENCHMARK \
       DRIFT_VEC=${DRIFT_VECTOR} \
       WIND_VEC=${WIND_VECTOR}


nsplug ${MISSIONS_DIR}/meta_vehicle.moos targ_$V1_NAME.moos -i -f \
       --path="${LAYOUT_DIR}:${MAP_DIR}:${MISSIONS_DIR}" \
       WARP=$TIME_WARP \
       LAT_ORIGIN=$LAT_ORIGIN \
       LONG_ORIGIN=$LONG_ORIGIN \
       VNAME=$V1_NAME \
       IP_ADDR="localhost" \
       V_MOOSDB=$V1_MOOSDB \
       PSHARE_PORT=$V1_PSHARE \
       SHORE_IP="localhost" \
       SHORE_PSHARE=$SHORE_PSHARE \
       VCOLOR=$V1_COLOR \
       START_POS="${V1_START_POS}" \
       GOAL_POS="${V1_GOAL_POS}" \
       PLANNER=$PLANNER \
       INTERMEDIATE_PTS="${NULL_PLANNER_PTS}" \
       SEARCH_BOUNDS="${SEARCH_BOUNDS}" \
       RANDOM_GUSTS=${RANDOM_GUSTS}

nsplug ${MISSIONS_DIR}/meta_vehicle.bhv targ_$V1_NAME.bhv -i -f \
       --path="${LAYOUT_DIR}:${MAP_DIR}:${MISSIONS_DIR}" \
       VNAME=$V1_NAME \
       START_POS="${V1_START_POS}" \
       USE_OBS_AVOID=$USE_OBS_AVOID \
       OPREGION_BOUNDS="${OPREGION_BOUNDS}"


if [ ${JUST_MAKE} = "true" ] ; then
    echo "Files assembled; nothing launched; exiting per request."
    rm -rf tmp
    exit 0
fi


#----------------------------------------------------------
#  Part 4: Launch the processes
#----------------------------------------------------------
# make export directory if it doesn't exist
mkdir -p "$(dirname $EXPORT_FILE)"

echo "Launching Shoreside MOOS Community. WARP is" $TIME_WARP
pAntler targ_shoreside.moos >& /dev/null &

echo "Launching $V1_NAME MOOS Community. WARP is" $TIME_WARP
pAntler targ_$V1_NAME.moos >& /dev/null &


if [ $GUI = "false" ] ; then
    sleep 5  # give everything time to boot up
    uPokeDB targ_shoreside.moos DEPLOY_ALL=true MOOS_MANUAL_OVERRIDE_ALL=false RESET_SIM_REQUESTED=all
fi

uMAC -t targ_shoreside.moos

if [ $GUI = "false" ] ; then
    sleep 5
    ktm  # kill any stragglers, seems to happen without the gui
fi

rm -rf tmp  # cleanup unzipped benchmark files

kill -- -$$
