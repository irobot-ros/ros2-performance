# Troubleshooting


The cross-compilation of ROS2 is a non trivial process. To tackle most of the possible issues, we decided to create a cross-compilation environment based on Docker.


## Environment setup issues


#### Linux error while building Dockerfile sysroot

While running `sysroots/get_sysroot.sh` or while building one of the provided Dockerfile you get the following error:

```
standard_init_linux.go:207: exec user process caused "exec format error"
```

**Possible reasons:** You are trying to build an ARM Docker image on an X86_64 machine. This requires running some ARM binaries which are not supported by default.

**Solution:** Follow the `Allow Docker to run and build ARM containers` instructions in [our Docker setup](../docker_setup).

## Cross-compilation issues

#### 0 packages finished

If when you run the cross-compilation script you get the following output message

```
Summary: 0 packages finished [0.16s]
[0.384s]
```

**Possible reasons:** This means that colcon has not found any package in the provided workspace.

**Solution:** Check that the workspace contains a `src` directory with the source code that you want to cross-compile. Moreover, it is important that the packages are actually present in the workspace and that they are not only symlinked.


#### Linker missing libPocoFoundation and or libyaml

While cross-compiling the ROS2 SDK you may get the following or a similar error

```
/usr/bin/arm-linux-gnueabihf-ld: warning: libPocoFoundation.so.50, needed by /root/ws/install/lib/librmw_implementation.so, not found (try using -rpath or -rpath-link)
/usr/bin/arm-linux-gnueabihf-ld: warning: libyaml.so, needed by /root/ws/install/lib/librcl_yaml_param_parser.so, not found (try using -rpath or -rpath-link)
/root/ws/install/lib/librcl_yaml_param_parser.so: undefined reference to `yaml_event_delete'
/root/ws/install/lib/librmw_implementation.so: undefined reference to `typeinfo for Poco::LibraryLoadException'
...
```

**Possible reasons:** the linker that you are using is looking for these libraries only in the sysroot, while they should be in the `install/lib` directory of the current workspace.

**Solution:** in the `.toolchain.cmake` file that you are using, add to the `CMAKE_C_FLAGS` and `CMAKE_CXX_FLAGS` a linker path to the current workspace library directory, for example `-Wl,-rpath-link=/root/ws/install/lib`.
If there are still problems, a work-around consists in cross-compiling up to these libraries and then adding them to the sysroot (a symbolic link is enough).


## Runtime issues

#### Not able to source a cross-compiled workspace

You try to source a cross-compiled workspace, but you get this error:

```
The build time path "/root/ws/install" doesn't exist.
Either source a script for a different shell or set the environment variable "COLCON_CURRENT_PREFIX" explicitly.
```

**Possible reasons:** the setup scripts inside the packages have hardcoded inside the path where they have been built. With cross-compilation, this path will become invalid once you move the workspace to your target platform.

**Solution:** follow the advice and set the `COLCON_CURRENT_PREFIX` environment variable to the current location of the `install` directory of the workspace.

```
export COLCON_CURRENT_PREFIX=<path_to_cross_compiled_ws>/install
```

#### ROS2 CLI not working, not able to run nodes

You try to run a ROS2 node using the standard CLI command

```
ros2 run examples_rclcpp_minimal_publisher publisher_lambda
```

However, you get a Python error similar to this one:

```
Failed to load entry point 'launch': No module named 'rclpy._rclpy'
```

**Possible reasons:** In order to use the CLI commands you need to have cross-compiled the ROS2 Python packages, that are ignored by the bash script in the `ignore_pkgs_scripts` directory since they require Python3.
Note that an alternative cause could be that you cross-compiled all the required packages, but the `PYTHON_SOABI` value specified in the `toolchain.cmake` file is wrong.

**Solution:** Besides cross-compiling all the Python packages and checking the `PYTHON_SOABI` value, there exists a quicker solution, suitable for these platforms where Python3 is not available.
Indeed the `install/lib/<package_name>` directory of a ROS2 workspace contains all the binaries of a specific package. The ROS2 CLI is just a convenient tool for not having to specify the full path to this binaries, but it is not required.



#### Runtime missing libPocoFoundation

You cross-compiled the ROS2 SDK, but when you run any executable you get the following error:

```
error while loading shared libraries: libPocoFoundation.so.50: cannot open shared object file: No such file or directory
```

**Possible reasons:** the `poco_vendor` package checks whether `Poco Foundation` is installed on your computer/sysrooot. If it finds it, it doesn't do anything, but if the library is not present, it installs it.
The installed library will be placed both in the ros2 workspace install directory as well as in the default library path of your computer/sysroot.
If you cross-compile the ROS2 SDK multiple times without restoring the sysroot, what happens is that after the first run the library is installed and included in the `install/lib` directory. However a second run will erase the workspace and this time the library will not be added to the workspace as CMake already finds it in the system. Thus if you copy this last workspace to the target board, it will not include the `libPocoFoundation` library.

**Solution:** you can generate again the sysroot, create a workspace containing only the [poco_vendor](https://github.com/ros2/poco_vendor) package and cross-compile it. Note that you don't need the ROS2 SDK for this operation.
Then copy the cross-compiled libraries to the target board.


#### SIGBUS error when running application

If your target is a armv7l (i.e. 32-bit Raspberry Pi or similar) there could be a byte alignment error during the cross-compilation.
This problem was present in the first release of ROS2 Crystal and then it has been addressed by [this PR](https://github.com/ros2/rcl/pull/365), which, after patch 1 has now been merged, so no further actions should be required.
If you have any problems, ensure that the `src/ros2/rcl` package is checking out version 0.6.4 or above.