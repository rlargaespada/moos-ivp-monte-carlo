#!/bin/bash -e

#^ NOTE: This script cannot be run directly, but must be
#^ called by a gen_benchmark_obs script from a child directory.
#^ It is placed at this level to avoid unnecessary duplication,
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
#  Part 2: Generate obstacle files for benchmarking
#----------------------------------------------------------

mkdir -p --verbose tmp

for i in {0..3}
do
    FILENAME=tmp/benchmark_obstacles_known_${i}.txt
    echo "writing to ${FILENAME}"
    gen_obstacles --poly=$RANDOM_OBS_REGION  --min_range=$RANDOM_OBS_MIN_RANGE    \
                  --max_size=$RANDOM_OBS_MAX_SIZE --min_size=$RANDOM_OBS_MIN_SIZE \
                  --amt=$RANDOM_OBS_AMT > $FILENAME
    sleep 1  # sleep for a bit so gen_obstacles gets a new random seed (based on sys time)
    FILENAME=tmp/benchmark_obstacles_unknown_${i}.txt
    echo "writing to ${FILENAME}"
    gen_obstacles --poly=$RANDOM_OBS_REGION  --min_range=$RANDOM_OBS_MIN_RANGE    \
                  --max_size=$RANDOM_OBS_MAX_SIZE --min_size=$RANDOM_OBS_MIN_SIZE \
                  --amt=$RANDOM_OBS_AMT > $FILENAME
    sleep 1  # sleep for a bit so gen_obstacles gets a new random seed (based on sys time)
done


#----------------------------------------------------------
#  Part 3: Compress obstacles files and remove originals
#----------------------------------------------------------
zip -v benchmark_obstacles.zip -m tmp/benchmark_obstacles*.txt
rmdir -p --verbose tmp
