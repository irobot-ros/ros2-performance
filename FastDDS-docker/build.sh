#!/bin/bash
#
# @author Alberto Soragna (asoragna at irobot dot com)
# @2018

docker pull ubuntu:18.04
docker build -t fastdds-test .
