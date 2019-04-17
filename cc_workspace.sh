#!/bin/bash

if [ -z "$1" ]; then
    echo "You must provide an absolute path to a workspace as positional argument!"
    echo "bash cc_workspace.sh /absolute/path/to/workspace"
    exit 1
fi

if [[ "$1" != /* ]]; then
    echo "You must provide an absolute path to a workspace, not a relative path:"
    echo "$1"
    exit 1
fi

if [ ! -d "$1" ]; then
    echo "The provided absolute path does not exist:"
    echo "$1"
    exit 1
fi

if [ -z "$TARGET_ARCHITECTURE" ]; then
    echo "Missing TARGET_ARCHITECTURE environment variables. Please run first"
    echo "source env.sh";
    exit 1;
fi

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

WORKSPACE_PATH=$1
SYSROOT_PATH="$THIS_DIR/sysroots/$TARGET_ARCHITECTURE"
TOOLCHAIN_PATH="$THIS_DIR/toolchains/generic_linux.cmake"
TOOLCHAIN_VARIABLES_PATH="$THIS_DIR/toolchains/"$TARGET_ARCHITECTURE".sh"

docker run -it \
    --volume $WORKSPACE_PATH:/root/ws \
    --volume $SYSROOT_PATH:/root/sysroot \
    --volume $TOOLCHAIN_PATH:/root/ws/toolchainfile.cmake \
    --volume $TOOLCHAIN_VARIABLES_PATH:/root/cc_export.sh \
    -w="/root/ws" \
    ros2_cc \
    /bin/bash -c 'source /root/.bashrc; \
        bash /root/compilation_scripts/cross_compile.sh'