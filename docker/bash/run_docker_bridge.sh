#!/bin/bash

docker run --privileged -d -it \
    -v /home/gs/workspace/metacam_nav/ros_bridge_ws:/mnt \
    --net=host                             \
    nav/rosbridge:app   