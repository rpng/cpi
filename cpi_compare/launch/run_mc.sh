#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
source /home/patrick/workspace/catkin_ws_vi/devel/setup.bash

# Loop through all datasets
for i in {00..49};
do

# Echo debug message
echo "BASH: running rawdata_$i";
start_time="$(date -u +%s)"

# Run our ROS launch file (note we send console output to terminator)
roslaunch cpi_compare synthetic_record.launch directory:="rawdata_$i" &> /dev/null

# Print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: $elapsed seconds to process rawdata_$i"


done
