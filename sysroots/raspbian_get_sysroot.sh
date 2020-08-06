#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

SYSROOT_TARGET=$THIS_DIR/$TARGET_ARCHITECTURE

# Docker-based get sysroot command
GET_SYSROOT_CMD="bash docker_based_sysroot.sh"
if ! $GET_SYSROOT_CMD; then
  echo "Error: failed to get sysroot for architecture $TARGET"
  exit 1
fi

# fix the sysroot ld.so.conf file removing the regex and manually inserting all the possibilities
# clear the file
> $SYSROOT_TARGET/etc/ld.so.conf

# copy included files
for filename in $SYSROOT_TARGET/etc/ld.so.conf.d/*; do
    cat ${filename} >> $SYSROOT_TARGET/etc/ld.so.conf
done
