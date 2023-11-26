#!/usr/bin/bash

set -e

cd "$(dirname "$0")"

./cmd_build.bash

source install/local_setup.bash

ros2 launch raisa_bringup raisa_launch.py