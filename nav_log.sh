#!/bin/bash

rosbag record /PannerAtuCloud /free_paths /tf /tf_static /speed /local_path  /stop /cmd_vel /odom_interface /move_base/GlobalPlanner/plan /move_base_simple/goal /way_point /move_base/global_costmap/costmap;exec bash;
