#!/bin/bash

docker run --privileged -d  \
    -v /home/gs/workspace/metacam_nav:/mnt \
    --net=host \
    --entrypoint /path/to/startup.sh \
    gs/melodic:latest