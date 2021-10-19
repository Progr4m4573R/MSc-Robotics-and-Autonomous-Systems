#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import radians
from math import degrees
class Mover:
    """
    A very simple Roamer implementation for Thorvald.
    It simply goes straight until any obstacle is within
    3 m distance and then just simply turns left.
    A purely reactive approach.
    """

    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to lasr scans and a Publisher to control
        the robot
        """
        self.publisher = rospy.Publisher(
            '/thorvald_001/teleop_joy/cmd_vel',
            Twist, queue_size=1)
        rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.callback)

    def callback(self, data):
        """
        Callback called any time a new laser scan becomes available
        """

        rospy.loginfo(
            rospy.get_caller_id() + "I heard %s", data.header.seq)
        min_dist = min(data.ranges)


        if data.ranges[320]  < 1:
            #if there is nothing in front
            t = Twist()
            t.linear.x = 0
            t.angular.z = radians(45);#rotate right at this speed
            print("Turning left to avoid obstacle...")
            self.publisher.publish(t)
            
        # #go forwards if there is nothing in front and priorities turning right
        elif data.ranges[320] > 1:
            t = Twist()
            t.linear.x = 0.5
            
            self.publisher.publish(t) 
            #self.image_callback
            print("Exploring....")

if __name__ == '__main__':
    rospy.init_node('mover')
    Mover()
    rospy.spin()

