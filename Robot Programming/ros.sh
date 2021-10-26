#!/bin/bash

gnome-terminal -- roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small
gnome-terminal -- rostopic list exec bash
gnome-terminal -- rqt_graph exec bash

echo 'Starting simulation...'
exit
