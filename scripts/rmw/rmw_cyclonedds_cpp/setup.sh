
#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

export CYCLONEDDS_URI=file://$THIS_DIR/zero-copy-shm.xml
# This should be done somewhere else so we can kill it more easily
#iox-roudi &

echo "Enablig shared memory zero-copy transport for cyclonedds"
