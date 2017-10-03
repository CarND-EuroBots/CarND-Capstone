#!/bin/bash
set -e

THIS_DIR="$(cd "$(dirname "$0")" && pwd -P && cd - > /dev/null)"
ROS_DIR="$THIS_DIR/ros"
DOCKER_IMAGE=eurobots/carnd_capstone:latest

docker pull "${DOCKER_IMAGE}"
docker run --rm=true --tty=true --volume="$ROS_DIR":"$ROS_DIR"        \
           --workdir="$ROS_DIR" --user=$(id -u):$(id -g)              \
           "${DOCKER_IMAGE}" /bin/bash -c                             \
           "source /opt/ros/kinetic/setup.bash; catkin_make"
