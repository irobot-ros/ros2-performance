#!/bin/bash
#
# @author Alberto Soragna (asoragna at irobot dot com)
# @2018

XSOCK=/tmp/.X11-unix

docker run -it --rm \
	  --net=host \
	  --privileged \
	  -e DISPLAY=$DISPLAY \
	  -v $XSOCK:$XSOCK \
	  -v $HOME/.Xauthority:/root/.Xauthority \
	  -v $PWD/workspace_SierraNevada:/root/workspace_SierraNevada \
	  -v $PWD/MultiProcess_SierraNevada:/root/MultiProcess_SierraNevada \
	  -v $PWD/scripts:/root/scripts \
	  fastdds-test \
	  bash
