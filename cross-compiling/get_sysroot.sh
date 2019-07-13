#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
SYSROOTS_DIR=$THIS_DIR/sysroots

if [ -z "$TARGET_ARCHITECTURE" ]; then
    echo "Missing TARGET_ARCHITECTURE environment variables. Please run first"
    echo "source env.sh";
    exit 1;
fi

# if we already have the sysroots for our architecture, remove it to create a new one
if [ -d $SYSROOTS_DIR/$TARGET_ARCHITECTURE ]; then
    echo "Removing existing sysroot $SYSROOTS_DIR/$TARGET_ARCHITECTURE to create a new one"
    rm -rf $SYSROOTS_DIR/$TARGET_ARCHITECTURE
fi

TARGET_FILE_NAME=$SYSROOTS_DIR/$TARGET_ARCHITECTURE"_get_sysroot.sh"

if [ -e $TARGET_FILE_NAME ]; then
  bash $TARGET_FILE_NAME
else
  echo "Error: the environment variable TARGET_ARCHITECTURE points to an unknown value:"
  echo "$TARGET_ARCHITECTURE"
  echo "Check the 'env.sh' script"
  exit 1
fi