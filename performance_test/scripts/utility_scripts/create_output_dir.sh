#!/bin/bash

## Check script arguments
if [ "$#" -ne 1 ]; then
    echo "Usage: ./create_output_dir.sh <OUTPUT_DIR_PATH>"
    exit 1
fi

OUTPUT_DIR_PATH=$1
OUTPUT_DIR_NAME=$(basename $OUTPUT_DIR_PATH)


## If $OUTPUT_DIR_PATH already exists, delete all its content
if [ -d "$OUTPUT_DIR_PATH" ]; then
  rm -rf $OUTPUT_DIR_PATH/*
else
  mkdir -p $OUTPUT_DIR_PATH
fi


