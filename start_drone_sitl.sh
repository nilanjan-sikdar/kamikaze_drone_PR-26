#!/bin/bash

# Ensure any stalled instances are erased before starting
pkill -9 -f sim_vehicle >/dev/null 2>&1
pkill -9 -f arducopter >/dev/null 2>&1
pkill -9 -f mavproxy >/dev/null 2>&1

ARDUPILOT_DIR="/home/nilanjan-sikdar/Downloads/ardupilot"
MY_WORKSPACE="/home/nilanjan-sikdar/Documents/PR'26/kamikaze_drone_PR-26"

source "$MY_WORKSPACE/venv/bin/activate"

cd "$ARDUPILOT_DIR"
# Workaround for ArduPilot's terminal launching bug with paths containing single quotes (')
cp "$MY_WORKSPACE/my_drone_params.parm" /tmp/my_drone_params.parm

echo "Launch ArduPilot SITL..."
python3 Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f webots-python \
    --console \
    --map \
    --add-param-file=/tmp/my_drone_params.parm
