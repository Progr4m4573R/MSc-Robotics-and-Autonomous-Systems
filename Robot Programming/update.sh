#!/bin/bash
# bash script for automating the installation of ROS melodic for linux computers
echo 'automating ros installation and updating'
#update system
sudo apt-get update && sudo apt-get upgrade
#installing ros melodic
sudo apt-get install ros-melodic-uol-cmp9767m-base ros-melodic-desktop
#source melodic to setup folder
source /opt/ros/melodic/setup.bash


exit
