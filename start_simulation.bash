#!/bin/bash
cd src/auv_gazebo/launch/
python3 bottom_obj.py
cd ../../..
source devel/setup.bash
roslaunch Simulation.launch