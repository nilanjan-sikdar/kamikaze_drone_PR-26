#!/bin/bash

export WEBOTS_CONTROLLER_URL="ipc://1234/tracker"
export WEBOTS_HOME=/snap/webots/current/usr/share/webots
export PYTHONPATH=$WEBOTS_HOME/lib/controller/python:$PYTHONPATH

ARDUPILOT_DIR="/home/nilanjan-sikdar/Downloads/ardupilot"
MY_WORKSPACE="/home/nilanjan-sikdar/Documents/PR'26/kamikaze_drone_PR-26"

source "$MY_WORKSPACE/venv/bin/activate"

echo "Connecting ArduPilot Webots bridge to 'tracker'..."
# Run the vehicle controller in the foreground
python3 "$ARDUPILOT_DIR/libraries/SITL/examples/Webots_Python/controllers/ardupilot_vehicle_controller/ardupilot_vehicle_controller.py" \
    --instance 0 \
    --camera camera \
    --camera-port 5599
