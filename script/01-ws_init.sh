#!/bin/sh

set -e

if [ ! -d "src" ]; then
	echo "Making missing \"src\" folder..."
	mkdir "$1/src"
fi

echo "Making catkin environment..."
catkin_make

echo "DONE!"
