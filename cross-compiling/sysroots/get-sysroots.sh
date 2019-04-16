#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
CROSS_COMPILATION_PATH=$THIS_DIR/..

if [ -z "$TARGET_ARCHITECTURE" ]; then
    echo "Missing TARGET_ARCHITECTURE environment variables. Please run first"
    echo "source env.sh";
    exit 1;
fi

# if we already have the sysroots for our architecture, remove it to create a new one
if [ -d $THIS_DIR/$TARGET_ARCHITECTURE ]; then
    rm -rf $THIS_DIR/$TARGET_ARCHITECTURE
fi

if [ "$TARGET_ARCHITECTURE" = "raspbian" ]; then

    # build a docker image of the target system
    docker build -t raspbian_sysroot -f $THIS_DIR/Dockerfile_raspbian $THIS_DIR

    # eventually remove old docker containers
    docker kill ros2-raspbian-sysroot
    docker rm ros2-raspbian-sysroot

    # create new docker container
    docker create --name ros2-raspbian-sysroot raspbian_sysroot

    # extract sysroot from docker container as tar file
    docker container export -o $THIS_DIR/$TARGET_ARCHITECTURE.tar ros2-raspbian-sysroot

    # uncompress sysroot
    chmod 777 $THIS_DIR/$TARGET_ARCHITECTURE.tar
    mkdir $THIS_DIR/$TARGET_ARCHITECTURE
    tar -xf $THIS_DIR/$TARGET_ARCHITECTURE.tar -C $THIS_DIR/$TARGET_ARCHITECTURE etc lib opt usr

    # remove tar file
    rm $THIS_DIR/$TARGET_ARCHITECTURE.tar

    # fix the sysroot ld.so.conf file removing the regex and manually inserting all the possibilities
    # clear the file
    > $THIS_DIR/$TARGET_ARCHITECTURE/etc/ld.so.conf

    # copy included files
    for filename in $THIS_DIR/$TARGET_ARCHITECTURE/etc/ld.so.conf.d/*; do
        cat ${filename} >> $THIS_DIR/$TARGET_ARCHITECTURE/etc/ld.so.conf
    done


else

    echo "Error: the environment variable TARGET_ARCHITECTURE points to an unknown value:"
    echo "$TARGET_ARCHITECTURE"
    echo "Check the 'env.sh' script"
    exit 1

fi
