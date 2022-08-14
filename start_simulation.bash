#!/bin/bash
cd
cd hydro/simulator/src/auv_gazebo/launch/
python3 bottom_obj.py
cd
cd hydro/simulator/
source devel/setup.bash
roslaunch Simulation.launch