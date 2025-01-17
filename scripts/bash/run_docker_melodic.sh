#!/bin/bash

docker run --privileged -d \
    -v /home/gs/workspace/metacam_nav:/mnt \
    --restart always                       \
    --net=host                             \
    --entrypoint /mnt/scripts/bash/start_docker_nav.sh \
    gs/melodic:latest   