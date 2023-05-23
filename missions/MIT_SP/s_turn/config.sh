#!/bin/bash

#----------------------------------------------------------
#  Part 1: Set Shoreside app config overwrites
#----------------------------------------------------------
# pMarineViewer initial window location
PMV_PAN_X="-160"
PMV_PAN_Y="-100"
PMV_ZOOM="0.8"

# random obstacle generation config
RANDOM_OBS_REGION="-89,-205:-89,-46:60,25:175,25:175,-205"
RANDOM_OBS_MIN_RANGE=10
RANDOM_OBS_MIN_SIZE=8
RANDOM_OBS_MAX_SIZE=15
RANDOM_OBS_AMT=7

# sim reset config
HDG_ON_RESET=135

#----------------------------------------------------------
#  Part 2: Set vehicle app config overwrites
#----------------------------------------------------------
SEARCH_BOUNDS="{-150,25:225,25:225,-205:-150,-205}"

# vehicle 1 app config parameters
V1_START_X="-125"; V1_START_Y="-75"
V1_START_POS="$V1_START_X,$V1_START_Y"
V1_GOAL_X="200"; V1_GOAL_Y="-75"
V1_GOAL_POS="$V1_GOAL_X,$V1_GOAL_Y"

NULL_PLANNER_PTS="25,-160:70,-15"
