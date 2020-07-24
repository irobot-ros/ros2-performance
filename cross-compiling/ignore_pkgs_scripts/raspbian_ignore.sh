#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

if [ -z "$1" ]; then
    echo "Working on $THIS_DIR workspace"
else
    echo "Moving to $1 workspace"
    cd $1
fi

echo "Ignoring packages..."

patch -p0 < $THIS_DIR/raspbian_fix_ros2_sources.patch
