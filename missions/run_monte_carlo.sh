#!/bin/bash -e
#----------------------------------------------------------
#  Script: run_monte_carlo.sh
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
        echo "  --export=<filestem>, -e=<filestem>               "
        echo "    CSV file to export simulation results to, without"
        echo "    the \".csv\" suffix. Defaults to \"$EXPORT_FILE\"."
        echo "    File is placed in the \"./${METRICS_DIR}/<planner>/\" directory."
        echo "  --no_obs_avoid                                   "
        echo "    Do not use pObstacleMgr obstacle avoidance behaviors during trials."
        echo "  --drift_vector=<heading>,<magnitude> or \"random\"  "
        echo "    Vector defining constant water currents affecting "
        echo "    obstacle and vehicle motion. Default is no drifts."
        echo "    Random drifts are generated with a random angle and"
        echo "    magnitude between 0 and 1 m/s."
        echo "  --wind_vector=<heading>,<magnitude> or \"random\"   "
        echo "    Vector defining cpnstant winds affecting vehicle motion"
        echo "    only. Default is no winds. Random winds are generated"
        echo "    with a random angle and magnitude between 0 and 1 m/s. "
        echo "  --random_gusts                                   "
        echo "    Add random varying gusts of wind disturbing vehicle"
        echo "    motion. Gusts are generated with a random direction"
        echo "    and a random magnitude between 0.5 and 2.      "
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
    elif [ "${ARGI::13}" = "--num_trials=" -o "${ARGI::3}" = "-n=" ] ; then
        NUM_TRIALS="${ARGI#*=}"
    elif [ "${ARGI::9}" = "--export=" -o "${ARGI::3}" = "-e=" ] ; then
        EXPORT_FILE="${ARGI#*=}"
        EXPORT_FILE=${EXPORT_FILE%.*}  # remove file extension if it exists

    elif [ "${ARGI}" = "--no_obs_avoid" ] ; then
        USE_OBS_AVOID="false"
    elif [ "${ARGI::15}" = "--drift_vector=" ] ; then
        DRIFT_VECTOR="${ARGI#*=}"
        if [ ${DRIFT_VECTOR,,} = "random" ] ; then
            DRIFT_TYPE="random"
            DRIFT_VECTOR=""
        else
            DRIFT_TYPE="given"
        fi
    elif [ "${ARGI::14}" = "--wind_vector=" ] ; then
        WIND_VECTOR="${ARGI#*=}"
        if [ ${WIND_VECTOR,,} = "random" ] ; then
            WIND_TYPE="random"
            WIND_VECTOR=""
        else
            WIND_TYPE="given"
        fi
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
# generate obstacle files
nsplug "${MAP_DIR}/${OBS_MAP_FILE}" "${OBS_CONST_FILE}" -i -f \
       --path="${LAYOUT_DIR}:${MAP_DIR}:${MISSIONS_DIR}" \
       MAP_BOUNDS="${MAP_BOUNDS}" \
       LAYOUT_OBSTACLES_FILE="${OBS_LAYOUT_FILE}"
gen_obstacles --poly=$RANDOM_OBS_REGION  --min_range=$RANDOM_OBS_MIN_RANGE    \
              --max_size=$RANDOM_OBS_MAX_SIZE --min_size=$RANDOM_OBS_MIN_SIZE \
              --amt=$RANDOM_OBS_AMT > "${OBS_KNOWN_FILE}"
sleep 1  # sleep for a bit so gen_obstacles gets a new random seed (based on sys time)
gen_obstacles --poly=$RANDOM_OBS_REGION  --min_range=$RANDOM_OBS_MIN_RANGE    \
              --max_size=$RANDOM_OBS_MAX_SIZE --min_size=$RANDOM_OBS_MIN_SIZE \
              --amt=$RANDOM_OBS_AMT > "${OBS_UNKNOWN_FILE}"


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
       USE_BENCHMARK=$USE_BENCHMARK \
       DRIFT_VEC=${DRIFT_VECTOR} \
       DRIFT_TYPE=$DRIFT_TYPE \
       WIND_VEC=${WIND_VECTOR} \
       WIND_TYPE=${WIND_TYPE}

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

kill -- -$$
