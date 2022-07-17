
#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

export CYCLONEDDS_URI=file://$PWD/zero-copy-shm.xml
iox-roudi &

echo "Enablig shared memory zero-copy transport for cyclonedds"
