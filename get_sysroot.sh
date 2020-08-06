#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
SYSROOTS_DIR=$THIS_DIR/sysroots

if [ -z "$TARGET_ARCHITECTURE" ]; then
    echo "Missing TARGET_ARCHITECTURE environment variables. Please run first"
    echo "source env.sh <TARGET_ARCHITECTURE>";
    exit 1;
fi

# if we already have the sysroots for our architecture, remove it to create a new one
if [ -d $SYSROOTS_DIR/$TARGET_ARCHITECTURE ]; then
    echo "Removing existing sysroot $SYSROOTS_DIR/$TARGET_ARCHITECTURE to create a new one"
    rm -rf $SYSROOTS_DIR/$TARGET_ARCHITECTURE
fi

# Make sure that we have a script for creating sysroot for target architecture
TARGET_FILE_NAME=$SYSROOTS_DIR/$TARGET_ARCHITECTURE"_get_sysroot.sh"
if [ ! -f $TARGET_FILE_NAME ]; then
  echo "Error: get_sysroot script not found for architecture $TARGET_ARCHITECTURE"
  echo "Looking for $TARGET_FILE_NAME"
  echo "Check the architecture name is correct and that the file exists"
  exit 1
fi

# Run the get sysroot script
GET_SYSROOT_CMD="bash $TARGET_FILE_NAME"
if ! $GET_SYSROOT_CMD; then
  echo "Error: failed to get sysroot for architecture $TARGET"
  exit 1
fi

if [ -d $SYSROOTS_DIR/$TARGET_ARCHITECTURE ]; then
    echo "Sysroot $SYSROOTS_DIR/$TARGET_ARCHITECTURE succesfully created"
else
    echo "Error: something went wrong while creating sysroot for $TARGET_ARCHITECTURE"
    echo "A sysroot was expected to be created at $SYSROOTS_DIR/$TARGET_ARCHITECTURE but is not there"
    exit 1
fi
