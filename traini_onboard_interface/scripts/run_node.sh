#! /bin/bash

set -e

STSL_BIN_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$STSL_BIN_DIR"

source ../../../setup.bash

if [ "$1" != "" ]
then
  export ROS_DOMAIN_ID=$1
else
  export ROS_DOMAIN_ID=0
fi

./traini_interface_node
