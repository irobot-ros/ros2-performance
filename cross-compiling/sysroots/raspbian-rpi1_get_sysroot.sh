#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# build a docker image of the target system
git clone https://github.com/raspberrypi/tools.git

# Get toolchain from:
# tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin/

# Get sysroot from:
# tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/arm-linux-gnueabihf/sysroot/

# PATCH: https://gcc.gnu.org/legacy-ml/gcc-patches/2014-03/msg01505.html
