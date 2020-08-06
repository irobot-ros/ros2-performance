#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

if [ -z "$TARGET_ARCHITECTURE" ]; then
    echo "Missing TARGET_ARCHITECTURE environment variables. Please run first"
    echo "source env.sh <TARGET_ARCHITECTURE>";
    exit 1;
fi

DOCKERFILE_NAME=$THIS_DIR/Dockerfile_$TARGET_ARCHITECTURE
DOCKER_IMAGE_NAME=$TARGET_ARCHITECTURE"_sysroot"
DOCKER_CONTAINER_NAME=ros2-$TARGET_ARCHITECTURE-sysroot
SYSROOT_TARGET=$THIS_DIR/$TARGET_ARCHITECTURE
SYSROOT_ARCHIVE=$SYSROOT_TARGET.tar

# build a docker image of the target system
docker build -t $DOCKER_IMAGE_NAME -f $DOCKERFILE_NAME $THIS_DIR

# eventually remove old docker containers
docker kill $DOCKER_CONTAINER_NAME
docker rm $DOCKER_CONTAINER_NAME

# create new docker container
docker create --name $DOCKER_CONTAINER_NAME $DOCKER_IMAGE_NAME

# extract sysroot from docker container as tar file
docker container export -o $SYSROOT_ARCHIVE $DOCKER_CONTAINER_NAME

# uncompress sysroot
chmod 777 $SYSROOT_ARCHIVE
mkdir $SYSROOT_TARGET
tar -xf $SYSROOT_ARCHIVE -C $SYSROOT_TARGET etc lib opt usr

# remove tar file
rm $SYSROOT_ARCHIVE
