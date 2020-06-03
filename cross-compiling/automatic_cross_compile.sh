# This script is intended to be used by the Jenkins night builds.
# But it still possible to use manually setting up this variables:
#
# TARGET=[raspbian]

# Set env variables
ROS2_DISTRO="${ROS2_DISTRO:=master}"

# Check that all variables are defined before start
if [[ -z "$TARGET" ]]
then
  echo "Error: environmental variables are not defined!"
  echo "Set values to TARGET"
  exit 1
fi

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
export BASE_DIR="$(dirname "$(pwd)")"

# Prepare cross-compiling environment
source $THIS_DIR/env.sh $TARGET

# Get sysroot
bash $THIS_DIR/get_sysroot.sh

# Remove ROS2 old cross-compilation workspace and get a new one
sudo rm -rf $BASE_DIR/ros2_cc_ws
bash $THIS_DIR/get_ros2_sources.sh --distro=$ROS2_DISTRO --ros2-path=$BASE_DIR/ros2_cc_ws

# Get HEAD of branch
HEAD=$(git ls-remote git://github.com/ros2/ros2 "$ROS2_DISTRO" | cut -c1-7)

# Create install directory for the cross-compilation results
TARGET_NAME=$(echo "$TARGET" | tr - _)
RESULTS_DIR=$BASE_DIR/ROS2_SDKs/ros2_"$ROS2_DISTRO"_"$TARGET_NAME"
echo "Create RESULTS_DIR=$RESULTS_DIR"
sudo rm -rf $RESULTS_DIR
mkdir -p $RESULTS_DIR

# Save the current packages versions and check if any changes
ROS2_SRCS_HEADS=$RESULTS_DIR/ros2.repos.by_commit
ROS2_SRCS_HEADS_PREV_RUN=$BASE_DIR/ros2_srcs_prev_run_"$TARGET_NAME"_"$ROS2_DISTRO".txt

vcs export --exact $BASE_DIR/ros2_cc_ws/src > $ROS2_SRCS_HEADS

if [ -f $ROS2_SRCS_HEADS_PREV_RUN ]; then
  diff -y  $ROS2_SRCS_HEADS $ROS2_SRCS_HEADS_PREV_RUN > $RESULTS_DIR/diff_ros2_versions.txt
fi
cp -R $ROS2_SRCS_HEADS $ROS2_SRCS_HEADS_PREV_RUN

# Ignore packages
bash $THIS_DIR/ignore_pkgs.sh $BASE_DIR/ros2_cc_ws $ROS2_DISTRO

# Run the cross-compilation and check the return code
CC_CMD="bash $THIS_DIR/cc_workspace.sh $BASE_DIR/ros2_cc_ws --no-it"
if $CC_CMD; then
  # If the build was successful, copy results to store as artifact
  cp -r $BASE_DIR/ros2_cc_ws/install/* $RESULTS_DIR
  cd $BASE_DIR/ROS2_SDKs/
  tar -czf ros2_"$ROS2_DISTRO"_"$TARGET_NAME".tar.gz ros2_"$ROS2_DISTRO"_"$TARGET_NAME"
else
  exit 1
fi
