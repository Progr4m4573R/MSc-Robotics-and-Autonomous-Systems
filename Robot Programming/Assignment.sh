#!/bin/bash

#Launches the grape counter script that countes grapes using the image camera and depth camera
python Assignment3cams.py
#Launches the grape wall follower script that looks for the nearest obstacle and circles it
python follow_grapes.py
#The grpe bunch locater that uses momemnts to drive the robot to the nearest grape bunch it sees.
python Autonomous_grape_searcher.py
