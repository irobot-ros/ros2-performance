#!/bin/bash

# build the cross compilation base docker image
docker build -t ros2_cc_base -f docker_environments/Dockerfile_base .

prefix="docker_environments/Dockerfile_"

# Build all the other dockerfiles in the docker_environments directory
for dockerfile_name in docker_environments/*
do
  # Check this is a Dockerfile (i.e. it starts with the prefix)
  if [[ $dockerfile_name != $prefix* ]]; then
    continue
  fi

  # Strip the prefix from the dockerfile_name
  platform_name=${dockerfile_name#"$prefix"}

  # Skip Dockerfile_base as this must be built first outside this loop
  if [ $platform_name != "base" ]; then
    docker build -t ros2_cc_$platform_name -f $dockerfile_name .
  fi
done

