#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash
source /uuv_simulator/devel/setup.bash
cd /simulator
source devel/setup.bash
python3 scripts/randomize_sauvc.py
roslaunch launch/sauvc.launch