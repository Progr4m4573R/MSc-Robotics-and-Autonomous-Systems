#!/usr/bin/env python
# Need to install jsk-visualisation library with 
#sudo apt-get install -y ros-melodic-jsk-visualization
# Please run rviz by rosrun rviz rviz -d `rospack find jsk_rviz_plugins`/config/pictogram.rviz
#
import rospy
import math
from jsk_rviz_plugins.msg import Pictogram, PictogramArray
from random import random, choice
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

def goal_callback(data):
    global cur_location, destination
    if data:
        destination = data
        cur_pose = rospy.wait_for_message("thorvald_001/odom", Odometry)
        cur_location = cur_pose.pose.pose.position
        

rospy.init_node("pictogram_object_demo_node")

cur_location = destination = Point()
p = rospy.Publisher("/pictogram_array", PictogramArray,  queue_size=1)
goal_sub = rospy.Subscriber("/robot_goal", Point, goal_callback)

r = rospy.Rate(1)
pictograms = ["fa-building","location"]

while not rospy.is_shutdown():
    
    arr = PictogramArray()
    arr.header.frame_id = "thorvald_001/odom"
    arr.header.stamp = rospy.Time.now()
    for index, character in enumerate(pictograms):
        msg = Pictogram()
        msg.header.frame_id = "thorvald_001/odom"
        msg.action = Pictogram.JUMP
        if index == 0:
            msg.header.stamp = rospy.Time.now()
            msg.pose.position = cur_location
        else:
            msg.header.stamp = rospy.Time.now()
            msg.pose.position = destination
        
        # It has to be like this to have them vertically orient the icons.
        msg.pose.orientation.w = 0.7
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = -0.7
        msg.pose.orientation.z = 0
        msg.size = 1
        msg.color.r = 25 / 255.0
        msg.color.g = 255 / 255.0
        msg.color.b = 240 / 255.0
        msg.color.a = 1.0
        msg.character = character
        arr.pictograms.append(msg)
        
    p.publish(arr)
    r.sleep()

