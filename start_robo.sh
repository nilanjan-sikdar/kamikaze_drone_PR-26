#!/bin/bash

# Setup environment for Webots extern controller
export WEBOTS_CONTROLLER_URL="ipc://1234/tracker"
export WEBOTS_HOME=/snap/webots/current/usr/share/webots
export PYTHONPATH=$WEBOTS_HOME/lib/controller/python:$PYTHONPATH

echo "Connecting robo_controller.py to Webots extern controller target 'robo'..."

# Activate python virtual environment if you need standard Python packages
source "/home/nilanjan-sikdar/Documents/PR'26/kamikaze_drone_PR-26/venv/bin/activate"

# Run the controller script
python3 robo_controller.py
