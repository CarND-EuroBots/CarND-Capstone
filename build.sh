#!/bin/bash
set -e

THIS_DIR="$(pwd)"
ROS_DIR="$THIS_DIR/ros"

docker run --rm=true --tty=true --volume="$ROS_DIR":"$ROS_DIR" \
           --workdir="$ROS_DIR" --user=$(id -u):$(id -g)       \
           eurobots/carnd_capstone /bin/bash -c                \
           "source /opt/ros/kinetic/setup.bash; catkin_make"
