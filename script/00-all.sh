#!/bin/sh

set -e

base="$HOME/git/robotics-2022"

echo "Moving to workspace: $base ..."
cd $base

for script in $base/script/*.sh; do
	if [ "$script" != "$base/script/00-all.sh" ]; then
		echo "Running $(basename $script)..."
		bash $script $base
	fi
done

echo "ALL DONE!"
