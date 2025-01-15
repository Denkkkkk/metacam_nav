docker run --privileged -it -d  \
    -v /home/gs/workspace/metacam_nav:/mnt \
    --net=host \
    gs/ros1:latest
