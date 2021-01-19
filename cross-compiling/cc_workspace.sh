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
    echo "source env.sh <TARGET_ARCHITECTURE_NAME>";
    exit 1;
fi

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
WORKSPACE_PATH=$1

# Some steps are not required for x86_64 compilation
if [ "$TARGET_ARCHITECTURE" != "x86_64" ]; then

    SYSROOT_PATH="$THIS_DIR/sysroots/$TARGET_ARCHITECTURE"
    TOOLCHAIN_TEMPLATE_PATH="$THIS_DIR/toolchains/generic_linux.cmake"
    TOOLCHAIN_ARCH_SPECIFIC_PATH="$THIS_DIR/toolchains/"$TARGET_ARCHITECTURE".sh"

    if [ ! -f "$TOOLCHAIN_TEMPLATE_PATH" ]; then
        echo "Generic toolchain not found. Looking for"
        echo "$TOOLCHAIN_TEMPLATE_PATH"
        exit 1
    fi

    if [ ! -f "$TOOLCHAIN_ARCH_SPECIFIC_PATH" ]; then
        echo "Toolchain settings for target architecture $TARGET_ARCHITECTURE not found. Looking for"
        echo "$TOOLCHAIN_ARCH_SPECIFIC_PATH"
        exit 1
    fi

    if [ ! -d "$SYSROOT_PATH" ]; then
        echo "Sysroot for target architecture $TARGET_ARCHITECTURE not found. Looking for"
        echo "$SYSROOT_PATH"
        echo "Please run first get_sysroot.sh"
        exit 1
    fi

    SYSROOT_VOLUME_ARG="--volume $SYSROOT_PATH:/root/sysroot"
    TOOLCHAIN_VOLUME_ARG="--volume $TOOLCHAIN_TEMPLATE_PATH:/root/ws/toolchainfile.cmake"
    TOOLCHAIN_ARCH_SPECIFIC_VOLUME_ARG="--volume $TOOLCHAIN_ARCH_SPECIFIC_PATH:/root/cc_export.sh"
    COMPILATION_SCRIPT=/root/compilation_scripts/cross_compile.sh
else
    COMPILATION_SCRIPT=/root/compilation_scripts/compile.sh
fi

INTERACTIVE_DOCKER_ARG="-it"
COMMAND=(/bin/bash -c "source /root/.bashrc && bash $COMPILATION_SCRIPT")

for i in "$@"
do
case $i in
    --no-it)
    INTERACTIVE_DOCKER_ARG=""
    shift
    ;;
    --debug)
    COMMAND=(bash)
    shift
    ;;
esac
done

CONTAINER_WORKSPACE_PATH=/root/ws
WORKSPACE_VOLUME_ARG="--volume $WORKSPACE_PATH:$CONTAINER_WORKSPACE_PATH"

docker run \
    $INTERACTIVE_DOCKER_ARG \
    $WORKSPACE_VOLUME_ARG \
    $SYSROOT_VOLUME_ARG \
    $TOOLCHAIN_VOLUME_ARG \
    $TOOLCHAIN_ARCH_SPECIFIC_VOLUME_ARG \
    -w="$CONTAINER_WORKSPACE_PATH" \
    ros2_cc_$TARGET_ARCHITECTURE \
    "${COMMAND[@]}"
