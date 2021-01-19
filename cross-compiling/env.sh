#!/bin/bash

if [ -z "$1" ]; then
    echo "Error! You have to provide a known architecture name"
    return 1
fi

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
TARGET_NAME=$1

#Check target architecture toolchain script
TOOLCHAIN_SCRIPT=$THIS_DIR/toolchains/$TARGET_NAME.sh
if [ ! -f "$TOOLCHAIN_SCRIPT" ] && [ "$TARGET_NAME" != "x86_64" ]; then
    echo "ERROR: File $TOOLCHAIN_SCRIPT does not exist!!!."
    return 1
fi

export TARGET_ARCHITECTURE=$TARGET_NAME
echo "Environment sourced correctly!"
echo "TARGET_ARCHITECTURE env variable set to '$TARGET_NAME'"
if [ "$TARGET_NAME" != "x86_64" ]; then
    echo "These paths will be used as target for the cross-compilation"
    echo "Path to sysroot: $THIS_DIR/sysroots/$TARGET_NAME"
    echo "Path to target specific toolchain: $TOOLCHAIN_SCRIPT"
fi
