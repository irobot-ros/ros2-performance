export TARGET_ARCH=arm

export CROSS_COMPILER_C="arm-linux-gnueabihf-gcc-6"
export CROSS_COMPILER_CXX="arm-linux-gnueabihf-g++-6"

export SYSROOT="/root/sysroot"
export PYTHON_SOABI="cpython-36m-arm-linux-gnueabihf"

#TODO: why the raspberry pi requires to specify rpaht-link? it may be due to the arm-linux-gnueabihf-gcc-6 compiler
#TODO: in the C_FLAGS we specify the numpy include path because this is built on the host and not on the sysroot.
# To be fixed, properly building numpy for python3.6
export TARGET_C_FLAGS="-mcpu=cortex-a7 -mtune=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -w -O2 -Wl,-rpath-link=/root/ws/install/lib -I /usr/local/lib/python3.6/dist-packages/numpy/core/include"
export TARGET_CXX_FLAGS="-mcpu=cortex-a7 -mtune=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -w -O2 -Wl,-rpath-link=/root/ws/install/lib"