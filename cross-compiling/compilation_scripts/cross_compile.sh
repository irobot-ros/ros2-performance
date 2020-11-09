#!/bin/bash

## script for cross-compiling the current workspace
## accepts as argument the path to the toolchain file to be used
## if no argument is provided, it looks for a default toolchain in the current directory.


if [ -z "$1" ]; then
    TOOLCHAIN_PATH=`pwd`/toolchainfile.cmake
else
    TOOLCHAIN_PATH=$1
fi

# clear out everything first
rm -rf build install log

ROS2_SETUP=/root/sysroot/setup.bash
if [ -f "$ROS2_SETUP" ]; then
    source /root/sysroot/setup.bash
fi

COLCON_CMD="colcon \
                build \
                --merge-install \
                --cmake-force-configure \
                --cmake-args \
                -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN_PATH \
                -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
                -DTHIRDPARTY=ON \
                -DBUILD_TESTING:BOOL=OFF"

# --merge-install is used to avoid creation of nested install dirs for each package

# Create exit code
EXIT_CODE=0

if ! $COLCON_CMD; then
  echo "Error: colcon command failed"
  EXIT_CODE=1
fi


# Set Read+Write+Execute permissions to workspace, to avoid
# having to be sudo to tar or remove the cross-compiled workspace
chmod -R a+rwx .

# Exit
exit $EXIT_CODE
