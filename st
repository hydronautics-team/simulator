#!/bin/bash
sleep 10
docker exec -d simulator-sauvc bash -c "source /opt/ros/humble/setup.bash && source /simulator/install/setup.bash && python3 /simulator/host_bridge/ws_bridge.py"
sleep 2
cd "/home/niksen/Desktop/New Folder/simulator/Exam" && ./untitled2
