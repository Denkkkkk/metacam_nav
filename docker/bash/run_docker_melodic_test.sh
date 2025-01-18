#!/bin/bash

docker run --privileged -d -it \
    -v /home/gs/workspace/metacam_nav:/mnt \
    --net=host                             \
    gs/melodic:latest                      \
    bash -c "cd /mnt && exec bash"