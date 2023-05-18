#!/bin/bash

# random obstacle generation config
RANDOM_OBS_REGION="-135,-200:-90,-50:45,17:250,17:250,-50:185,-200"
RANDOM_OBS_MIN_RANGE=10
RANDOM_OBS_MIN_SIZE=8
RANDOM_OBS_MAX_SIZE=15
RANDOM_OBS_AMT=7


# pMarineViewer initial window location
PMV_PAN_X="10"
PMV_PAN_Y="-140"
PMV_ZOOM="0.65"


# vehicle 1 app config parameters
V1_START_X="-100"; V1_START_Y="-220"
V1_START_POS="$V1_START_X,$V1_START_Y"
V1_GOAL_X="210"; V1_GOAL_Y="30"
V1_GOAL_POS="$V1_GOAL_X,$V1_GOAL_Y"
