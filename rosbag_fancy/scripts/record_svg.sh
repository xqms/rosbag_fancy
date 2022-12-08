#!/bin/bash

# This script is used to record the SVGs shown on GitHub.

set -e

export ROS_MASTER_URI=http://localhost:11411

# Kill child processes on exit
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

roscore -p 11411 &

sleep 1

rostopic pub -r 20.0 /tf tf2_msgs/TFMessage "{ transforms: [] }" &
rostopic pub -r 0.2 /test tf2_msgs/TFMessage "{ transforms: [] }" &



type() {
	str="$1"
	for (( i=0; i<${#str}; i++ )); do
		echo -ne "${str:$i:1}"
		sleep 0.05
	done
}

wget -nc 'https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b0-2014-07-11-10-58-16.bag'

rm -f play.cast
rm -f out.bag

export PS1="max@machine:~$ "
(
	sleep 0.1

	type $'rosbag_fancy record -o out.bag /tf /test /misspelled'
	sleep 0.5
	type $'\n'

	sleep 5

	killall -SIGINT rosbag_fancy

	sleep 1

	######################### INFO

	type $'rosbag_fancy info out.bag'
	sleep 0.5
	type $'\n'

	sleep 5

	echo 'clear'

	######################## PLAY

	sleep 0.5
	type $'rosbag_fancy play b0-2014-07-11-10-58-16.bag\n'
	sleep 5.0

	# Right arrow
	type $'\e[C'
	sleep 0.5
	type $'\e[C'
	sleep 0.5
	type $'\e[C'

	sleep 2.0

	# Left arrow
	type $'\e[D'
	sleep 0.5
	type $'\e[D'
	sleep 0.5
	type $'\e[D'

	sleep 2.0

	# Pause
	type ' '

	sleep 2.0

	# Resume
	type ' '

	sleep 2.0

	# Exit
	type 'q'

	echo 'exit'
) | asciinema rec --stdin -c "bash --norc" play.cast
