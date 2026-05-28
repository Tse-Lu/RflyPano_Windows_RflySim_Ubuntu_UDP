#!/bin/bash

WORKSPACE_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

gnome-terminal --tab --title="1. Fisheye & Pinhole Images" -- bash -c "
    source $WORKSPACE_PATH/devel/setup.bash;
    roslaunch rflysim_ros_pkg four_fisheye.launch;
    exec bash"

sleep 1.0

gnome-terminal --tab --title="2. IMU Data" -- bash -c "
    source $WORKSPACE_PATH/devel/setup.bash;
    roslaunch rflysim_ros_pkg imu.launch;
    exec bash"

sleep 1.0

gnome-terminal --tab --title="3. Panoramic Stitching" -- bash -c "
    source $WORKSPACE_PATH/devel/setup.bash;
    roslaunch rflysim_4fisheye_2_pano_pkg four_fisheye_2_pano.launch;
    exec bash"


