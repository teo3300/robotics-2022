#!/bin/sh

set -e

if [ "$1/src" != "$(echo $ROS_PACKAGE_PATH | cut -d':' -f1)" ]; then
	echo "Sourcing workspace..."
    source devel/setup.bash
fi

echo "DONE!"
