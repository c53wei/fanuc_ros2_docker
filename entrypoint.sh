#!/bin/bash

set -e

echo "Hello from entrypoint.sh"

source /opt/ros/humble/setup.bash
source ~/ws_fanuc/install/setup.bash

# Execute the command passed to the entrypoint
exec "$@"