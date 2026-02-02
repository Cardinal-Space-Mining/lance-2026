#!/bin/bash

mem_kb=$(awk '/MemTotal/ {print $2}' /proc/meminfo)

if [ "$mem_kb" -le 8196044 ];   # less than or equal to ~8GB RAM
then
    echo "USING LIMITED THREADS DUE TO LOW RAM AMOUNT"
    export MAKEFLAGS="-j 1"
fi

case "$1" in
    --la) ROBOT_TARGET="-DROBOT_TARGET=ALL" ;;
    --l1) ROBOT_TARGET="-DROBOT_TARGET=LANCE1" ;;
    --l2) ROBOT_TARGET="-DROBOT_TARGET=LANCE2" ;;
esac

START_TIME=$(date +%s.%N)

colcon build \
    --symlink-install \
    --executor parallel \
    --event-handlers console_direct+ \
    --packages-ignore lance \
    --cmake-args \
        -Wno-dev \
        -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON

colcon build \
    --symlink-install \
    --event-handlers console_direct+ \
    --packages-select lance \
    --cmake-args \
        -Wno-dev \
        -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON \
        ${ROBOT_TARGET}

END_TIME=$(date +%s.%N)
DELTA_TIME=$(echo "$END_TIME - $START_TIME" | bc)

echo ">> Build finished in $DELTA_TIME seconds."
