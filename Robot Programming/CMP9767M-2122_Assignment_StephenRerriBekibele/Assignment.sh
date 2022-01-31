#!/bin/bash
#bash script for automating the launch of Robot Programming Assignment 1

assignment_version = $1
echo 'Launching Assignment scripts'


if [ $assignment_version =='1' ]; then
    #Launches the grape counter script that countes grapes using the image camera and depth camera
    gnome-terminal -- python Assignment3cams.py
    #Launches the grape wall follower script that looks for the nearest obstacle and circles it
    gnome-terminal -- python Follow_grapes.py
fi

if [ $assignment_version =='2' ]; then
    #Launches the grape counter script that countes grapes using the image camera and depth camera
    gnome-terminal -- python Assignment3cams.py
    #The grape bunch locater that uses momemnts to drive the robot to the nearest grape bunch it sees.
    gnome-terminal -- python Autonomous_grape_searcher.py
fi

exit


