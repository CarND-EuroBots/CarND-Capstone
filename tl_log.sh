#!/bin/bash
set -e

THIS_DIR="$(cd "$(dirname "$0")" && pwd -P && cd - > /dev/null)"
ROS_DIR="$THIS_DIR/ros"

# Re-build the code
echo "Building the code..."
$THIS_DIR/build.sh

if [ "$#" -eq 1 ]
then
    # Launch simulator if it's not running yet
    if pgrep "system_integr" > /dev/null
    then
        echo "Great, the simulator is already running!"
    else
        echo "Starting the simulator..."
        $THIS_DIR/ros/src/styx/unity_simulator_launcher.sh
    fi

    # Launch the ROS nodes
    LOG_DIR_BASE="$(cd "$(dirname "$1")" && pwd -P && cd - > /dev/null)"
    LOG_DIR_NAME="$(basename "$1")"
    LOG_DIR_FULL="$LOG_DIR_BASE/$LOG_DIR_NAME"

    docker run --rm=true --tty=true --interactive=true               \
               --user=$(id --user):$(id --group)                     \
               --volume="/tmp":"/.ros"                               \
               --volume="$THIS_DIR":"$THIS_DIR"                      \
               --workdir="$THIS_DIR"                                 \
               --network=host                                        \
               --env DISPLAY                                         \
               --volume /tmp/.X11-unix:/tmp/.X11-unix                \
               eurobots/carnd_capstone /bin/bash -c                  \
               "source /opt/ros/kinetic/setup.bash;
                source ros/devel/setup.bash;
                roslaunch ros/launch/tl_gt_log.launch log_dir:=$LOG_DIR_FULL"

else
    echo "Usage: $0 <path_to_log_directory>"
fi
