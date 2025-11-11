#!/bin/bash
set -e
# Set ROS network configuration for host network mode
echo export ROS_IP="127.0.0.1" >> /home/${USERNAME}/.bashrc
echo export ROS_HOSTNAME="localhost" >> /home/${USERNAME}/.bashrc
echo export DISABLE_ROS1_EOL_WARNINGS="True"  >> /home/${USERNAME}/.bashrc
echo source "/opt/ros/noetic/setup.bash"  >> /home/${USERNAME}/.bashrc
source "/opt/ros/noetic/setup.bash"
cd "/home/ros/catkin_ws"
catkin_make
echo source "/home/ros/catkin_ws/devel/setup.bash" >> /home/${USERNAME}/.bashrc
cd "/home/ros"
source "/home/${USERNAME}/.bashrc"
exec "$@"


# # Define custom functions to control the tmule
# export TMULE_FILE=${BASE_WS}/src/auto_shepherd_simulation_ros2/tmule/launcher.tmule.yaml
# function s(){  tmule -c $TMULE_FILE -W 3 launch ; }
# function t(){  tmule -c $TMULE_FILE terminate ; }
# function r(){  tmule -c $TMULE_FILE -W 3 relaunch ; }
