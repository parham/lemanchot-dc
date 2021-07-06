#!/bin/bash

# Build the docker image
docker build -t lemanchot-dc:v0.1 .
xhost +
# Run the docker image
mkdir -p ./recordings
docker run -it --rm -e DISPLAY="unix:0.0" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/.Xauthority:/root/.Xauthority \
        --privileged \
        --net=host \
        --gpus all \
        --mount type=bind,source="${PWD}/recordings",target=/home/lemanchot-dc/recordings \
        -w /home/lemanchot-dc/
        lemanchot-dc:v0.1

# RUN command for when running on MANIFOLD 2-C
# docker run -it --rm -e DISPLAY="unix:0.0" \
#         -v /tmp/.X11-unix:/tmp/.X11-unix \
#         -v $HOME/.Xauthority:/root/.Xauthority \
#         --privileged \
#         --net=host \
#         --mount type=bind,source="${PWD}/recordings",target=/home/lemanchot-dc/recordings \
#         -w /home/lemanchot-dc/
#         lemanchot-dc:v0.1

xhost -
