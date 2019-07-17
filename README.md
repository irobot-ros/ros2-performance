# ROS2 cross-compilation tools

This directory contains Docker-based tools for cross-compiling ROS2 packages for embedded platforms.

If you are interested in additional resources about cross-compilation, check the [**ROS2 Cross-compilation official guide**](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation) or the references at the bottom of this page.

A list of common cross-compilation issues and solutions is provided in the [troubleshooting page](troubleshooting.md).

The supported ROS2 distributions are:

 - `ardent`
 - `bouncy`
 - `crystal`
 - `dashing`


The supported architectures are:

 - `raspbian`


## Requirements

 - Docker

If you don't have Docker installed follow [this link](../docker_setup.md).
Ensure that you have support for building and running ARM Docker containers.

## Build


    bash build.sh

This will build the `Dockerfile` which provides a Docker environment with all the cross-compilation dependencies installed.

## How to use it

You can use these tools to cross-compile the ROS2 SDK or specific packages. Note that in the second case, you need an already cross-compiled SDK which will have to be copied to the sysroot of your target platform.

We are currently not able to cross-compile all the ROS2 package, due to their dependencies.
That's not a problem as the missing ones are mostly visualization-related packages.
You can find the list of the packages to ignore for each specific ROS2 release [here](ignore_pkgs_scripts).

**NOTE:** the cross-compilation script mounts the workspace that you want to cross-compile as a Docker volume. This does not go well with symbolic links. For this reason ensure that the workspace contains the whole source code and not a symlink to the repositories.

In the following sections you can find some examples about how to cross-compile different types of packages.
Note that no changes are required if you want to cross-compile packages for a different ROS2 distribution.
However it's recommended to wipe out the sysroot and get a new one when you want to cross-compile for a new distribution.

## Cross-compile ROS2 SDK

**NOTE:** there is an automated script for cross-compiling specific versions of the ROS2 SDK.
For example you can run:

```
export TARGET=raspbian
export ROS2_DISTRO=dashing
bash automatic_cross_compile.sh
```

You can find the output in the directory `install/"$ROS2_BRANCH"_"$HEAD"_"$TARGET"`.

Let's go through an example showing how to manually cross-compile ROS2 Crystal SDK for `raspbian` board.

Source the environment variables for this architecture using

    source env.sh raspbian

If you need a sysroot for the architecture, create it.

    bash get_sysroot.sh

This command will download a sysroot for the archicture specified by the `TARGET_ARCHITECTURE` environment variable (the argument passed to the `env.sh` script above, `raspbian` in this case).
Note that if you already have a sysroot with the same name inside the `sysroots` directory, it will be overwritten by the new one.

If you want to use your own sysroot, instead of generating a new one, you can skip the last instruction and just place your sysroot in the `sysroots` directory. Your sysroot directory must be renamed to `raspbian` or as the specified `TARGET_ARCHITECTURE` that you passed to `env.sh`.

Create a ROS2 workspace containing the sources you need

    mkdir -p ~/ros2_cc_ws/src
    cd ~/ros2_cc_ws
    wget https://raw.githubusercontent.com/ros2/ros2/crystal/ros2.repos
    vcs import src < ros2.repos

Ignore some packages and python dependencies (note that there is a different script for each distribution you want to  cross-compile)

    cd -
    bash ignore_pkgs.sh ~/ros2_cc_ws dashing

Cross-compile the workspace

    bash cc_workspace.sh ~/ros2_cc_ws

The provided workspace will be mounted as a volume and it will be populated with the output of the cross-compilation.

#### Install

Copy the `install/lib` directory from the cross-compiled SDK to your target system.
```
cd ~/ros2_cc_ws
scp -r install/lib user@address:~/ros2_crystal
```

## Cross-compile ROS2 workspaces

Let's go through an example showing how to cross-compile some ROS2 Crystal packages for `raspbian` board.

Source the environment variables for this architecture using

    source env.sh raspbian

If you have followed the instructions in the previous section, you will already have a sysroot.

If you need a sysroot for the architecture, create it.

    bash get_sysroot.sh

Note that if you want to use your own sysroot, instead of generating a new one, you can skip the last instruction and just place your sysroot in the `sysroots` directory. The name of the sysroot directory must match the name be `raspbian`.

If you are creating a new sysroot or if your sysroot does not contain the ROS2 SDK, you must add it.
You can follow the instructions in the previous section

Copy the SDK into the sysroot

    cp -R ~/ros2_cc_ws/install/share/* sysroots/raspbian/usr/share/
    cp -R ~/ros2_cc_ws/install/lib/* sysroots/raspbian/usr/lib/
    cp -R ~/ros2_cc_ws/install/include/* sysroots/raspbian/usr/include/

Note that after adding contents to the sysroot, you can always revert it back to its original state by running again the `get-sysroots.sh` script.

Create a ROS2 workspace, for example containing example nodes.

    mkdir -p ~/my_ros2_cc_ws/src
    svn checkout https://github.com/ros2/examples/trunk/rclcpp ~/my_ros2_cc_ws/src/examples_rclcpp

Now you can cross-compile the workspace

    bash cc_workspace.sh ~/my_ros2_cc_ws

The provided workspace will be mounted as a volume and it will be populated with the output of the cross-compilation.


#### Install

Copy the `install/lib` directory from the cross-compiled workspace to your target system.

Prefer using `rsync` to copy over files since `scp` fails randomly. For example,

```
cd ~/my_ros2_cc_ws
scp -r install/lib user@address:~/ros2_crystal
```

Note that if you place the libraries in a place different from the `usr/lib` directory, you will have to specify export their path.
For example:

    export LD_LIBRARY_PATH=~/ros2_crystal:$LD_LIBRARY_PATH


## Add support for additional target architectures

One of the goals of this cross-compilation framework is to be quite easy to be extended to new target architectures.

For example, if you want to add a target named `my_architecture` you need the following:

 - A compilation environment for your architecture. You have to create a new Dockerfile named `docker_environments/Dockerfile_my_architecture`. This Dockerfile must begin with the line
    ```
    FROM ros2_cc_base
    ```

    Then it has only to install the compiler for your architecure.
 - A bash script that sets compilation flags for the toolchain named `toolchains/my_architecture.sh`
 - A sysroot named `sysroots/my_architecture` or a script `sysroots/my_architecture_get_sysroot.sh` to generate it.


## References

 - [ROS2 cross-compilation official tools](https://github.com/ros2/cross_compile)
 - [ROS2 step by step cross-compilation on ARM](https://github.com/ros2-for-arm/ros2/wiki/ROS2-on-arm-architecture)
 - [ROS2 RaspberryPi cross-compilation](https://github.com/alsora/ros2-raspberrypi)
