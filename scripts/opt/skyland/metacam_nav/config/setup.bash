#!/bin/bash

script_dir=$(cd "$(dirname "$0")" && pwd)

# config rosbridge topic
python3 ${script_dir}/rosbridge_cfg.py
sleep 0.1s
systemctl restart metacam_ros2_wrapper.service

# config blind param
python3 ${script_dir}/blind_cfg.py