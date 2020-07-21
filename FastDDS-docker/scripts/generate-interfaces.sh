#!/bin/bash

# Intended to be run in a Docker container provided
# with dependencies

# Generate messages from IDL format
cd /root/SingleProcess_SierraNevada/src/idl
/root/Fast-DDS-Gen/scripts/fastddsgen *.idl -d ../msg

# Compile
cd /root/SingleProcess_SierraNevada/build
cmake ..
make

# Run
./sierra_nevada
