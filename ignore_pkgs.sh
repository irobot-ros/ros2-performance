#!/bin/bash

if [ -z "$1" ]; then
  echo "You must specify a workspace directory!"
  exit 1
fi

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

workspace_directory=$1
echo "Working on $workspace_directory workspace"

# You can specify a distribution, otherwise the default one will be used
distribution="dashing"
if [ -z "$2" ]; then
  echo "Using default $distribution ignore"
else
  distribution=$2
  echo "Using $distribution ignore"
fi

# Run the ignore script for the specific distribution
bash $THIS_DIR/ignore_pkgs_scripts/$distribution"_ignore.sh" $workspace_directory

# Eventually run the ignore script for the specific platform
platform_specific_script=$THIS_DIR/ignore_pkgs_scripts/$TARGET_ARCHITECTURE"_ignore.sh"
if [ -e $platform_specific_script ]; then
  bash $platform_specific_script $workspace_directory
fi

