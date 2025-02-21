#!/bin/bash

docker run --privileged -d \
    -v /home/gs/workspace/metacam_nav:/mnt \
    --name=metacam_nav                     \
    --restart always                       \
    --net=host                             \
    --entrypoint /mnt/docker/bash/start_docker_nav.sh \
    gs/melodic:latest   