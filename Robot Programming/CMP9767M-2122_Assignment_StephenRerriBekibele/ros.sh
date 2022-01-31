#!/bin/bash

#bash script for automating the launch of ros melodic for linux computers

gnome-terminal -- roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small
gnome-terminal -- rostopic list exec bash
gnome-terminal -- rqt_graph exec bash

echo 'Starting simulation...'
exit
