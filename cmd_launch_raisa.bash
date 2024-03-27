#!/usr/bin/bash

set -e # Exit immediately if a command exits with a non-zero status.
cd "$(dirname "$0")" # Change directory to the script location.

./cmd_build.bash
source ./install/setup.bash
ros2 launch raisa_bringup raisa.launch.py
