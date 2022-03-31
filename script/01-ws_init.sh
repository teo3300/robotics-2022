#!/bin/sh

set -e

base="$HOME/git/robotics-2022"

if [ ! -d "src" ]; then
	echo "Making missing \"src\" folder..."
	mkdir "$base/src"
fi

echo "Making catkin environment..."
catkin_make

echo "DONE!"
