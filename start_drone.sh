#!/bin/bash

# Ensure any stalled instances are erased before starting
pkill -9 -f sim_vehicle >/dev/null 2>&1
pkill -9 -f arducopter >/dev/null 2>&1
pkill -9 -f mavproxy >/dev/null 2>&1
pkill -9 -f ardupilot_vehicle_controller >/dev/null 2>&1

echo "Launch ArduPilot SITL for Webots..."
export WEBOTS_CONTROLLER_URL="ipc://1234/tracker"
export WEBOTS_HOME=/snap/webots/current/usr/share/webots
export PYTHONPATH=$WEBOTS_HOME/lib/controller/python:$PYTHONPATH

# We use the ardupilot repo in Downloads
ARDUPILOT_DIR="/home/nilanjan-sikdar/Downloads/ardupilot"
MY_WORKSPACE="/home/nilanjan-sikdar/Documents/PR'26/kamikaze_drone_PR-26"

# Activate Python environment
source "$MY_WORKSPACE/venv/bin/activate"

# 1. Run the Webots python controller that talks to SITL
# We run this in the background (&) so it keeps running while sim_vehicle starts
python3 "$ARDUPILOT_DIR/libraries/SITL/examples/Webots_Python/controllers/ardupilot_vehicle_controller/ardupilot_vehicle_controller.py" \
    --instance 0 \
    --camera camera \
    --camera-port 5599 &

# Add a small delay to make sure the controller starts properly
sleep 2

# 2. Run sim_vehicle.py
cd "$ARDUPILOT_DIR"
# Workaround for ArduPilot's terminal launching bug with paths containing single quotes (')
cp "$MY_WORKSPACE/my_drone_params.parm" /tmp/my_drone_params.parm

python3 Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f webots-python \
    --console \
    --map \
    --add-param-file=/tmp/my_drone_params.parm
