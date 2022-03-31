#!/bin/sh

set -e

base="$HOME/git/robotics-2022"

for script in $base/script/*; do
	if [ "$script" != "$base/script/00-all.sh" ]; then
		echo "Running $(basename $script)..."
		bash "$script"
	fi
done

echo "ALL DONE!"
