#!/bin/bash

# This script is used to record the GIFs shown on GitHub.

set -e

export ROS_MASTER_URI=http://localhost:11411

# Kill child processes on exit
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

roscore -p 11411 &

sleep 1

rostopic pub -r 20.0 /tf tf2_msgs/TFMessage "{ transforms: [] }" &
rostopic pub -r 0.2 /test tf2_msgs/TFMessage "{ transforms: [] }" &

wget -nc 'https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b0-2014-07-11-10-58-16.bag'

rm -f out.bag

vhs < play.tape
