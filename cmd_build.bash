#!/usr/bin/bash

set -e # Exit immediately if a command exits with a non-zero status.
cd "$(dirname "$0")" # Change directory to the script location.

colcon --log-base /dev/null build --symlink-install --event-handlers console_direct+ desktop_notification-