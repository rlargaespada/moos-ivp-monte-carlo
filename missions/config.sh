#!/bin/bash

# launch script argument defaults
TIME_WARP=1
JUST_MAKE="false"
GUI="true"

PLANNER="dstarlite"; PLANNER_OPTIONS=("nullplanner" $PLANNER  "gcs" "gcsr")
NUM_TRIALS=3
EXPORT_FILE="default"

USE_OBS_AVOID="true"
DRIFT_STRENGTH=0
DRIFT_DIR="x"; DRIFT_DIR_OPTIONS=($DRIFT_DIR "y" "random")

# shoreside app config parameters
SHORE_MOOSDB="9000"
SHORE_PSHARE="9200"

META_OBS_CONST_FILE="meta_obstacles_const.txt"
OBS_CONST_FILE="targ_obstacles_const.txt"
OBS_KNOWN_FILE="targ_obstacles_known.txt"
OBS_UNKNOWN_FILE="targ_obstacles_unknown.txt"

# vehicle 1 app config parameters
V1_NAME="artemis"
#? where do i list what goes where? declare everything here, leave values unset? high level defaults?
V1_COLOR="red"
V1_MOOSDB="9100"
V1_PSHARE="9300"
