#!/bin/bash

#----------------------------------------------------------
#  Part 1: Set launch script argument defaults
#----------------------------------------------------------
TIME_WARP=1
JUST_MAKE="false"
GUI="true"

PLANNER="dstarlite"; PLANNER_OPTIONS=("nullplanner" $PLANNER  "gcs" "gcsr")
NUM_TRIALS=3
EXPORT_FILE="default"

USE_OBS_AVOID="true"
DRIFT_STRENGTH=0
DRIFT_DIR="x"; DRIFT_DIR_OPTIONS=($DRIFT_DIR "y" "random")

METRICS_DIR="metrics"
USE_BENCHMARK="false"

#----------------------------------------------------------
#  Part 2: Set Shoreside app config defaults
#----------------------------------------------------------
# datum
LAT_ORIGIN="43.825300"
LONG_ORIGIN="-70.330400"
MAP_BOUNDS="{-159,91:-159,-357:276,-357:276,91}"

# ports
SHORE_MOOSDB="9000"
SHORE_PSHARE="9200"

# pMarineViewer config
TIFF_FILE="forrest19.tif"
PMV_PAN_X="10"
PMV_PAN_Y="-140"
PMV_ZOOM="0.65"

# random obstacle generation parameters
RANDOM_OBS_REGION="-135,-200:-90,-50:45,17:250,17:250,-50:185,-200"
RANDOM_OBS_MIN_RANGE=10
RANDOM_OBS_MIN_SIZE=8
RANDOM_OBS_MAX_SIZE=15
RANDOM_OBS_AMT=7

# obstacle meta and target files
OBS_MAP_FILE="meta_obstacles_map.txt"
OBS_LAYOUT_FILE="meta_obstacles_layout.txt"
OBS_CONST_FILE="targ_obstacles_const.txt"
OBS_KNOWN_FILE="targ_obstacles_known.txt"
OBS_UNKNOWN_FILE="targ_obstacles_unknown.txt"

# sim reset config
HDG_ON_RESET="relative"

#----------------------------------------------------------
#  Part 3: Set vehicle app config defaults
#----------------------------------------------------------
SEARCH_BOUNDS="{-150,50:-150,-250:250,-250:250,50}"

# vehicle 1 app config parameters
V1_NAME="artemis"
V1_COLOR="red"
V1_MOOSDB="9100"
V1_PSHARE="9300"

V1_START_X="-100"; V1_START_Y="-220"
V1_START_POS="$V1_START_X,$V1_START_Y"
V1_GOAL_X="210"; V1_GOAL_Y="30"
V1_GOAL_POS="$V1_GOAL_X,$V1_GOAL_Y"

OPREGION_BOUNDS="${MAP_BOUNDS}"
NULL_PLANNER_PTS=""
