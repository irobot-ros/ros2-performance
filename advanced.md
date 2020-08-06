# Advanced cross-compilation

This readme contains some more advanced cross-compilation features provided by this framework.


## Add new ROS 2 packages to be cross-compiled

You should always first start from a standard ROS 2 workspace, e.g. obtained from

```
bash get_ros2_sources.sh --distro=foxy --ros2-path=~/ros2_cc_ws
bash ignore_pkgs.sh ~/ros2_cc_ws foxy
```

Then you can get the ROS 2 packages you are interested in and clone them into the `src` directory of the workspace.

Before cross-compiling, you have to make sure that the dependencies of your new packages are satisfied.

#### ROS 2 dependencies

If your ROS 2 package `A` depends on a ROS 2 package `B`, you have to make sure that `B` is also in the `src` directory of your workspace.

Note that if `B` is a standard ROS 2 package, i.e. it is part of the [ROS 2 repositories](https://github.com/ros2/ros2/blob/master/ros2.repos), it will already be in the workspace.
However, you still have to make sure that it is not being currently ignored.
We are ignoring packages because they may be not required for use on embedded targets, but your new package may have a dependency on them.
You can see the complete list of ignored packages with:

```
find ~/ros2_cc_ws/src | grep COLCON_IGNORE
```

To stop ignoring a package, just remove the `COLCON_IGNORE` file in its directory.
You may also want to update the ignore scripts to permanently stop ignoring the package.
The packages to be ignored in a workspace are specified in two different scripts: a distribution-specific script such as `ignore_pkgs_scripts/<ROS2_DISTRO>_ignore.sh` and an architecture-specific script such as `ignore_pkgs_scripts/<TARGET_ARCHITECTURE>_ignore.sh`.

If you stop ignoring a package, you will have to consider it as any other new ROS 2 package and make sure that its dependencies are satisfied.


#### Generic (non-ROS) dependencies

If your ROS 2 package `A` depends on a non-ROS package `B`, you have to make sure that `B` is contained in the sysroot of your target architecture.

Different approaches can be taken, depending on how you are generating the sysroot for your target architecture.
The generation procedure of the architecture can be deduced from the `sysroots/<TARGET_ARCHITECTURE>_get_sysroot.sh` script.

If the sysroot is generated from a Dockerfile, the simplest solution consists in editing the `sysroots/Dockerfile_<TARGET_ARCHITECTURE>` file to add your new dependencies there.

If that is not possible, e.g. because you are downloading an already generated sysroot and you don't have access to an environment where you can easily add new packages to it, you will need to manually cross-compile these dependencies.
The procedure will vary a lot depending on the nature of the package and on the target architecture, the following is an example for TinyXML2, a CMake based package.
The following snippet can be added to the `sysroots/<TARGET_ARCHITECTURE>_get_sysroot.sh` script.
It will require the cross-compiler installed on your system.

```
# export compilation variables
source $CROSS_COMPILATION_PATH/toolchains/$TARGET_ARCHITECTURE.sh
# hack: overwrite the sysroot path since the one just sourced is valid in the dockerfile
export SYSROOT=$CROSS_COMPILATION_PATH/sysroots/$TARGET_ARCHITECTURE

# add tinyxml2 dependency to the sysroot
git clone https://github.com/leethomason/tinyxml2
mkdir -p tinyxml2/build && cd $_
cmake -DCMAKE_TOOLCHAIN_FILE=$CROSS_COMPILATION_PATH/toolchains/generic_linux.cmake ..
make

# go back to starting directory
cd -
# copy the built libraries and headers to the sysroot
cp tinyxml2/build/libtinyxml2.so* $THIS_DIR/$TARGET_ARCHITECTURE/lib/
cp tinyxml2/tinyxml2.h $THIS_DIR/$TARGET_ARCHITECTURE/usr/include/

# clean directory
rm -rf tinyxml2
```

Note that if your newly-added dependency is not being statically linked to the ROS 2 packages, you will have to install it on your target system as well.

### Applying patches

It may happen that the ROS 2 package that you downloaded is not suitable for being cross-compiled as it is.
If the cross-compilation fails, you may want to address the failure by applying a patch to the failing ROS 2 package.

You can either fork the ROS 2 package and apply the patch to your version or you can keep using the standard version and let the cross-compilation framework to apply the patch.

In the second case, you will need to add to `ignore_pkgs_scripts/<TARGET_ARCHITECTURE>_ignore.sh` something like
```
patch -p0 < $THIS_DIR/<TARGET_ARCHITECTURE>_my_changes.patch
```

Where `ignore_pkgs_scripts/<TARGET_ARCHITECTURE>_my_changes.patch` contains the patch that you want to apply.

## Debugging of cross-compilation issues

When you add new ROS 2 packages to your cross-compilation workspace, it's common that you will face compilation errors.
The terminal will show WARN level logs.
You can find all logs and all build commands in the directory `ros2_cc_ws/log/latest_build/<ROS2_PACKAGE>`.

To speed-up debugging, it is recommended to manually invoke the build commands.
The `--debug` option will bring you inside the cross-compilation Docker environment

```
bash cc_workspace.sh ~/ros2_cc_ws --debug
```

To start a build, **from within the Docker environment**
```
bash /root/compilation_scripts/cross_compile.sh
```

You can modify the cross_compile script `/root/compilation_scripts/cross_compile.sh` **from within the Docker environment** to customize your debugging.
Common changes are the following:

 - Comment the line that clears the workspace every time to speed up builds `rm -rf build install log`
 - Rebuild only subset of packages, adding one of the following to the `colcon build` arguments: `--packages-up-to MY_PACKAGE` or `--packages-select MY_PACKAGE` or `--packages-start LAST_SUCCESSFUL_PACKAGE`.
 - Change the colcon log verbosity adding `--log-level debug`

## Cross-compile individual ROS 2 packages

If for any reason you don't want to cross-compile a whole ROS 2 workspace made of all the ROS 2 core packages, but you only want to cross-compile a specific individual package, you can do that by first cross-compiling the standard ROS 2 SDK, as described in the [README](README.md), and then you can copy the cross-compiled libraries into the sysroot of your target architecture.

```
cp -R ~/ros2_cc_ws/install/cmake sysroots/<TARGET_ARCHITECTURE>/usr/cmake
cp -R ~/ros2_cc_ws/install/share/* sysroots/<TARGET_ARCHITECTURE>/usr/share/
cp -R ~/ros2_cc_ws/install/lib/* sysroots/<TARGET_ARCHITECTURE>/usr/lib/
cp -R ~/ros2_cc_ws/install/include/* sysroots/<TARGET_ARCHITECTURE>/usr/include/
```

**NOTE:** calling `bash get_sysroot.sh` will erase the current sysroot and generate a new one from scratches, thus losing the copied ROS 2 SDK.

Then you can just cross-compile your workspace. Since ROS 2 libraries have been copied to standard locations in the sysroot, `colcon build` will be able to find them.


## Add support for additional target architectures

One of the goals of this cross-compilation framework is to be easy to be extended to new target architectures.

If you want to add a target named `my_architecture` you will need the following:

 - A compilation environment for your architecture. You have to create a new Dockerfile named `docker_environments/Dockerfile_my_architecture`. This Dockerfile must begin with the line
    ```
    FROM ros2_cc_base
    ```

    Then it has only to install the compiler for your architecure or any additional, architecture specific tools required.
 - A bash script that sets compilation flags for the toolchain named `toolchains/my_architecture.sh`, this will be used to fill the variables in the [generic toolchain](toolchains/generic_linux.cmake)
 - A sysroot named `sysroots/my_architecture` or a script `sysroots/my_architecture_get_sysroot.sh` to generate it.
