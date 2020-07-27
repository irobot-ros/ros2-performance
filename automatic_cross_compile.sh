# This script is intended to be used by the Jenkins night builds.
# But it still possible to use manually setting up this variables:
#
# TARGET=[raspbian]

# Set env variables
ROS2_DISTRO="${ROS2_DISTRO:=master}"

# Check that all variables are defined before start
if [[ -z "$TARGET" ]]
then
  echo "Error: target architecture is not specified!"
  echo "Set environment variable TARGET"
  echo "export TARGET=raspbian"
  exit 1
fi

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
BASE_DIR="$(dirname "$(pwd)")"

# Prepare cross-compiling environment
source $THIS_DIR/env.sh $TARGET

if [[ -z "$WORKSPACE_DIR" ]]; then
  # If the user has not specified a workspace directory, try to create one
  # Make sure that workspace directory does not exist before running
  WORKSPACE_DIR=$BASE_DIR/ros2_"$ROS2_DISTRO"_"$TARGET_NAME"_ws
  if [ -d "$WORKSPACE_DIR" ]; then
    echo "Error: workspace directory $WORKSPACE_DIR already exists."
    echo "Remove it and run again the script."
    exit 1
  fi

  # Get ROS 2 sources
  echo "Create WORKSPACE_DIR=$WORKSPACE_DIR"
  bash $THIS_DIR/get_ros2_sources.sh --distro=$ROS2_DISTRO --ros2-path=$WORKSPACE_DIR
else
  # Use the user-provided workspace directory
  echo "Using WORKSPACE_DIR=$WORKSPACE_DIR"
fi

# Ignore packages
bash $THIS_DIR/ignore_pkgs.sh $WORKSPACE_DIR $ROS2_DISTRO

# Get sysroot
GET_SYSROOT_CMD="bash $THIS_DIR/get_sysroot.sh"
if ! $GET_SYSROOT_CMD; then
  echo "Error: failed to get sysroot for architecture $TARGET"
  exit 1
fi

# Create install directory for the cross-compilation results
TARGET_NAME=${TARGET#*-}
RESULTS_DIR=$BASE_DIR/ROS2_SDKs/ros2_"$ROS2_DISTRO"_"$TARGET_NAME"
if [ -d "$RESULTS_DIR" ]; then
  echo "Error: results directory $RESULTS_DIR already exists."
  echo "Remove it and run again the script."
  exit 1
fi
echo "Create RESULTS_DIR=$RESULTS_DIR"
mkdir -p $RESULTS_DIR

# Save the current packages versions
ROS2_SRCS_HEADS=$RESULTS_DIR/ros2.repos.by_commit
vcs export --exact $WORKSPACE_DIR/src > $ROS2_SRCS_HEADS

# Run the cross-compilation and check the return code
CC_CMD="bash $THIS_DIR/cc_workspace.sh $WORKSPACE_DIR --no-it"
if ! $CC_CMD; then
  echo "Error: the cross-compilation step failed"
  exit 1
fi

# If the build was successful, copy results to store as artifact
cp -r $WORKSPACE_DIR/install/* $RESULTS_DIR
cd $BASE_DIR/ROS2_SDKs/
tar -czf ros2_"$ROS2_DISTRO"_"$TARGET_NAME".tar.gz ros2_"$ROS2_DISTRO"_"$TARGET_NAME"
