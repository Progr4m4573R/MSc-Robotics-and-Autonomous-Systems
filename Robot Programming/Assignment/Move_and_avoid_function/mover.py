#!/usr/bin/env python
#!/usr/bin/env python3
#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#import grape_detection
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
        to listen to laser scans and a Publisher to control
        the robot
        """
        self.publisher = rospy.Publisher(
            '/thorvald_001/teleop_joy/cmd_vel',
            Twist, queue_size=1)
        rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.callback)
        
        #Calls image processing node
        #grape_detection.Camera()

    def callback(self, data):
        """
        Callback called any time a new laser scan becomes available
        """
        t = Twist()
        rospy.loginfo(
            rospy.get_caller_id() + " I heard %s", data.header.seq)
        min_dist = min(data.ranges)
        
        safe_distance = min(data.ranges)# gets the minimum of the ranges from laser sensor
        
        #if there is something in front turn clockwise to avoid
        if safe_distance < 0.1:# in metres?

            t.angular.z = 1;#rotate left 
            print("Turning left  to avoid obstacle...")
            
        # #go forwards if there is nothing in front 
        elif safe_distance > 1:
            t.linear.x = 0
            print("Exploring....")




if __name__ == '__main__':
    rospy.init_node('mover')
    Mover()
    rospy.spin()

