#!/bin/bash


if [ -z "$1" ]; then
    echo "Error! You have to provide a known architecture name"
    return 1
else
    TARGET_NAME=$1
    echo "TARGET_ARCHITECTURE set to '$TARGET_NAME'"
fi

export TARGET_ARCHITECTURE=$TARGET_NAME

#Check target architecture toolchain script
THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
TOOLCHAIN_SCRIPT=$THIS_DIR/toolchains/$TARGET_ARCHITECTURE.sh
if [ ! -e "$TOOLCHAIN_SCRIPT" ]; then
    echo "ERROR: File $TOOLCHAIN_SCRIPT does not exist!!!."
    return 1
else
    echo "Environment sourced correctly!"
    echo "These paths will be used as target for the cross-compilation"
    echo "Path to sysroot: $THIS_DIR/sysroots/$TARGET_ARCHITECTURE"
    echo "Path to target specific toolchain: $THIS_DIR/toolchains/$TARGET_ARCHITECTURE.sh"
    return 0
fi