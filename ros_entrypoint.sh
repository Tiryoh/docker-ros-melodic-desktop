#!/bin/bash
set -e

# setup ros environment
touch ~/.bash_aliases && echo "function catkin_make(){(cd ~/catkin_ws && command catkin_make \$@) && source ~/catkin_ws/devel/setup.bash;}" >> ~/.bash_aliases

if [ $# -gt 1 ]; then
	echo $@ | /bin/bash -li
else
	exec bash -li -c "$@"
fi
