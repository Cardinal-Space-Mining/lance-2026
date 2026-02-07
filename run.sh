#!/bin/bash

usage() {
    echo "Usage:"
    echo "  $0 [options] [launch args]"
    echo ""
    echo "Options:"
    echo "  --canbus    Run canbus bringup and shutdown before/after ros nodes"
    echo "  --local     Use cyclonedds middleware with localhost-only discovery"
    echo "  --help      Print this message"
    echo ""
    echo "Launch Args:"
    echo "> Dynamic config reassignments of the form KEY:=VALUE"
    echo "> See lance/config/lance.json for main configuration."
    exit 1
}

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

source "$SCRIPTPATH/../install/setup.bash"

RUN_CANBUS=false
LOCAL_ONLY_ROS=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --canbus) RUN_CANBUS=true; shift ;;
        --local) LOCAL_ONLY_ROS=true; shift ;;
        --help) usage ;;
        *) break ;;
    esac
done

if [[ "$LOCAL_ONLY_ROS" == true ]]; then
    # Set env vars to change dds and disable net discovery
    export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
    export CYCLONEDDS_URI="file://$SCRIPTPATH/cyclonedds.xml"
fi

# Run the can bus bringup script if enabled
if [[ "$RUN_CANBUS" == true ]]; then
    "$SCRIPTPATH/phoenix-driver/scripts/can_bringup.sh"
fi

# Base ROS2 launch command
BASE_CMD=(
    ros2
    launch
    lance
    lance.launch.py
)
"${BASE_CMD[@]}" "$@"

# Run the can bus shutdown script if bringup was run
if [[ "$RUN_CANBUS" == true ]]; then
    "$SCRIPTPATH/phoenix-driver/scripts/can_shutdown.sh"
fi
