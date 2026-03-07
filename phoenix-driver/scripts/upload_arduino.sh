#!/bin/bash
set -e

echo "Detecting Arduino USB device..."

# Known Arduino Leonardo vendor:product ID
ARDUINO_ID="2341:8036"

DEVICE=""

for DEV in /dev/ttyACM*; do
    [ -e "$DEV" ] || continue

    USB_PATH=$(udevadm info -q path -n "$DEV")
    USB_INFO=$(udevadm info -q all -p "$USB_PATH")

    VID=$(echo "$USB_INFO" | grep ID_VENDOR_ID= | cut -d'=' -f2)
    PID=$(echo "$USB_INFO" | grep ID_MODEL_ID= | cut -d'=' -f2)

    DEVICE_ID="$VID:$PID"

    if [ "$DEVICE_ID" == "$ARDUINO_ID" ]; then
        DEVICE="$DEV"
        echo "Arduino detected at $DEVICE"
        break
    fi
done

if [ -z "$DEVICE" ]; then
    echo "ERROR: No compatible Arduino detected."
    exit 1
fi

# Resolve script location
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

SKETCH_PATH="$SCRIPTPATH/../src/arduino_motor_reset"
FQBN="arduino:avr:leonardo"

echo "Building and uploading sketch..."
arduino-cli compile \
    --upload \
    -p "$DEVICE" \
    --fqbn "$FQBN" \
    "$SKETCH_PATH"

echo "Upload complete ✅"
