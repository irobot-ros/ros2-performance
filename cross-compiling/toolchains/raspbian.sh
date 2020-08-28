export TARGET_ARCH=arm

export CROSS_COMPILER_C="arm-linux-gnueabihf-gcc-8"
export CROSS_COMPILER_CXX="arm-linux-gnueabihf-g++-8"

export SYSROOT="/root/sysroot"
export PYTHON_SOABI="cpython-37m-arm-linux-gnueabihf"

export TARGET_C_FLAGS="-mcpu=cortex-a7 -mtune=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -w -O2"
export TARGET_CXX_FLAGS="-mcpu=cortex-a7 -mtune=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -w -O2"
