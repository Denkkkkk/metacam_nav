#!/bin/bash
sleep 2s

source /opt/ros/humble/setup.bash
source /opt/ros/humble/setup.sh
source /opt/ros/humble/share/rclpy/local_setup.bash

python3 /home/gs/workspace/metacam_nav/src/Top_service/websockets/scripts/host_send.py
