#!/bin/bash

#----------------------------------------------------------
#  Part 1: Set Shoreside app config overwrites
#----------------------------------------------------------
# pMarineViewer initial window location
PMV_PAN_X="0"
PMV_PAN_Y="280"
PMV_ZOOM="0.65"

# random obstacle generation config
RANDOM_OBS_REGION="-125,-275:-125,-175:-95,-75:200,-75:200,-275"
RANDOM_OBS_MIN_RANGE=10
RANDOM_OBS_MIN_SIZE=8
RANDOM_OBS_MAX_SIZE=15
RANDOM_OBS_AMT=7

#----------------------------------------------------------
#  Part 2: Set vehicle app config overwrites
#----------------------------------------------------------
SEARCH_BOUNDS="{-125,-325:-125,-75:200,-75:200,-325}"

# vehicle 1 app config parameters
V1_START_X="-75"; V1_START_Y="-300"
V1_START_POS="$V1_START_X,$V1_START_Y"
V1_GOAL_X="175"; V1_GOAL_Y="-300"
V1_GOAL_POS="$V1_GOAL_X,$V1_GOAL_Y"
