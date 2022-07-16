
#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

export FASTRTPS_DEFAULT_PROFILES_FILE=$THIS_DIR/zero-copy-shm.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

echo "Enablig shared memory zero-copy transport for fastrtps"
