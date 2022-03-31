#!/bin/sh

set -e

base="$HOME/git/robotics-2022"

if [ "$base/src" != "$(echo $ROS_PACKAGE_PATH | cut -d':' -f1)" ]; then
	echo "Sourcing workspace..."
fi

echo "DONE!"
