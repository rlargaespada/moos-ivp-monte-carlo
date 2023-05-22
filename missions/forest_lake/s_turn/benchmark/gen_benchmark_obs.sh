#!/bin/bash -e

# define upper level param directories
LAYOUT_DIR=$(dirname $PWD)  # layout directory is one level up
MAP_DIR=$(dirname $LAYOUT_DIR)  # map directory is two levels up
MISSIONS_DIR=$(dirname $MAP_DIR)  # missions directory is three levels up

# call template script from missions dir
source ${MISSIONS_DIR}/run_gen_benchmark_obs.sh
