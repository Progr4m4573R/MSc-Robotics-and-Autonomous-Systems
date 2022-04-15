#!/usr/bin/env python
from glob import glob
from turtle import pu

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from math import cos, sin
from tf import TransformListener, transformations
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np
from math import radians
import robot_front_camera
import robot_right_camera
import robot_left_camera
#import Follow_grapes
#This program is based of the CMP9767M mover with pose workshop example. University of Lincoln CMP9767M wiki.Available at: https://github.com/LCAS/CMP9767M. [Accessed on: 27/1/2022]
class Mover:
    object_location = None

    def __init__(self):
        #created and imported some optional scripts to enable the robots cameras for viewing.
        # I cannot have all three simultaneously active as this can crash the cv.imshow function
        #robot_front_camera.image_projection()
        #robot_left_camera.image_projection()
        #robot_right_camera.image_projection()
        
        #Created a publiser to send geometry twist messages to the thorvald robot and enable it to move.
        self.publisher = rospy.Publisher(
            '/thorvald_001/teleop_joy/cmd_vel',
            Twist, queue_size=1)
            #Created a subscriber to listen to the incoming data from the thorvold_001 front laserscan topic 
        rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.laser_callback)
        self.pose_pub = rospy.Publisher(
            '/nearest_obstacle',
            PoseStamped,queue_size=1
        )
        #created a lister of type transform listener to retrive the transformations of the robot's pose.
        self.listener = TransformListener()

        #Library Used to convert from ROS images to OpenCV
        self.bridge = cv_bridge.CvBridge()
        #created a subscriber to retrieve the incoming image data from the thorvald robot
        rospy.Subscriber("/thorvald_001/kinect2_front_camera/hd/image_color_rect",
            Image, self.image_callback)

        #self.object_cords_sub = rospy.Subscriber("/thorvald_001/object_location",PoseStamped, self.object_coords)
        #use to make the robot move
        self.twist = Twist()

    def laser_callback(self, data):
        """
        Callback called any time a new laser scan becomes available
        """

        rospy.logdebug("I heard %s", data.header.seq)
        #set a minimum safe distance to prevent the robot crashing into obstacles
        min_dist = min(data.ranges)

        t = self.twist

        # If anything is closer than 4 metres anywhere in the
        # scan, we turn away
        if min_dist < 2:
            self.wall_follower()
            print("Avoiding obstacle...")
        else:  # if all obstacles are far away, let's keep 
            # moving forward at 0.8 m/s
            t.linear.x = 0.8

        self.publisher.publish(t)
        #gets the number of ranges of scans from the laser scanner and returns the min of the value
        index_min = min(
            range(len(data.ranges)),
            key=data.ranges.__getitem__)
        
        alpha = data.angle_min + (index_min * data.angle_increment)
 
        laser_point_2d = [ 
            cos(alpha) * min_dist, 
            sin(alpha) * min_dist,
            0.0]

        pose = PoseStamped()

        pose.header = data.header

        # fill in the slots from the points calculated above.
        # bit tedious to do it this way, but hey... 
        pose.pose.position.x = laser_point_2d[0]
        pose.pose.position.y = laser_point_2d[1]
        pose.pose.position.z = laser_point_2d[2]

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(alpha/2)
        pose.pose.orientation.w = cos(alpha/2)

        self.pose_pub.publish(pose)

        rospy.loginfo(
            "The closest point in laser frame coords is at\n%s"
            % pose.pose.position
            )

        transformed_pose = self.listener.transformPose("thorvald_001/base_link", pose)
        rospy.loginfo(
            "The closest point in robot coords is at\n%s"
            % transformed_pose.pose.position
            )
    def image_callback(self,camera_info_msg):
        t=Twist()
        cv2.namedWindow("Front_camera",1)
        #Below i apply Morphological Transformations to remvoe excess data from my image 
    
        #created two arrays, one big and one small to store the data after a mask open and close operation is done
        # if kernel is too big then the blobs wont be detected
        self.kernelOpen=np.ones((10,10))# uses two techniques called dialation and erosion to open and image an filter out noise to increase the accuracy of the mask
        self.kernelClose=np.ones((25,25))
        #after converting from ros image to open cv2 image and then to hsv to make the grapes easier to pick out
        image = self.bridge.imgmsg_to_cv2(camera_info_msg,desired_encoding='bgr8')
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        #Lastly, I use the in.Range function to filter out every colour but purple
        # detect a grape in the color image
        image_mask = cv2.inRange(hsv, (100,30,55), (255,255,255))

        #Morphological transformations
        self.maskClose=cv2.morphologyEx(image_mask,cv2.MORPH_CLOSE,self.kernelClose)

        self.maskOpen=cv2.morphologyEx(self.maskClose,cv2.MORPH_OPEN,self.kernelOpen)
       
        #conts stores the number of contours that are detected in the image
        maskFinal=self.maskOpen
        _,conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #I use the drawCOntours function to draw a rectangle around all contours that re detected in the image and within my purple mask
        cv2.drawContours(image,conts,-1,(255,0,0),1)
        for i in range(len(conts)):
            x,y,w,h=cv2.boundingRect(conts[i])
            cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,255), 2)# we draw a box around each contour 
            cv2.putText(image, str(i+1),(x,y+h),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0))#count the number of contours there are
            
        #counts number of bounding boxes on screen    
        print(len(conts), " bunches of grapes have been detected")
        
        h, w, d = image.shape
        gM = cv2.moments(image_mask)
        #I use my grape mask to create a cx and cy variable which where the mask is detected
        if gM['m00'] > 0:
            print("grape detected!")
            cx = int(gM['m10']/gM['m00'])
            cy = int(gM['m01']/gM['m00'])
            cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1)
            #i use the difference between the width of the image and w of the camera to create an eror variable
            err = cx - w/2
            self.twist.linear.x = 0.5
            #this error variable is used to correct the robot's angle so it remains facing the mask
            self.twist.angular.z = -float(err) / 1000
            print (t.angular.z)
            print("moving... via moments to grape")
            #i then publish a moment of 0.5m/s to the robot while it does this to make thorvald move to grapes
            self.publisher.publish(self.twist)
        else:#M is not greater than 0 so the robot moves aimlessly without direction.  
            #"moving... w/o moments")
            return True
        cv2.imshow("Front_camera", image)
        cv2.imshow("Final Mask", maskFinal)
        cv2.waitKey(3)

    def robot_control(self):
        #call image callback
        self.image_callback(self,self.camera_info_msg)
        if self.image_callback()==True:
            self.laser_callback(self,self.incoming_data.ranges[320])

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
                                                            #Wall following algorithm 
    def wall_follower(self):
        #This main function allows the robot to do different actions depending on the state it is in.     
        global pub_ 
        
        pub_ = self.publisher
        rospy.Subscriber('/thorvald_001/front_scan', LaserScan, clbk_laser)
        
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            msg = Twist()
            if state_ == 3:
                msg = find_grapes()
                print("Looking for grapes")
            elif state_ == 1:
                msg = turn_left()
                print("turning left")
            elif state_ == 0:
                msg = follow_the_grapes()
                print("Following grape wall")
            elif state_ == 2:
                msg = turn_right()
                print("Turning right")
                pass
            else:
                rospy.logerr('Unknown state!')
            pub_.publish(msg)
            
            rate.sleep()
        print("Following grape wall...")

    # Define variables for the directions we want the robot to be able to turn
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
#This state variable allows us to switch between the states we define in stae_dict_
state_ = 0
state_dict_ = {
    0: 'find the grapes',
    1: 'turn left',
    2: 'follow the grapes',
    3: 'turn right',
}
    #Created a laser callback to recieve the data from the laser sensor
def clbk_laser(msg):
    global regions_
    regions_= {
        'right': min(min(msg.ranges[0:143]), 10),
        'fright':min(min(msg.ranges[144:287]), 10),
        'front':min(min(msg.ranges[288:431]), 10),
        'fleft':min(min(msg.ranges[432:575]), 10),
        'left':min(min(msg.ranges[576:713]), 10),
    }
    take_action()
    #This state variable determines how the robot will switch between each possible state 
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print('Grape follower - [%s] - %s' %(state,state_dict_[state]))
        state_ = state
#This function decides what actions the robot takes in the differnt states
def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    d = 1.5
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

#A Function to enable the robot to drive around until a grape wall is detected.
def find_grapes():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.1
    return msg
#We define an action for each state we want the robot to be able to be in.
def turn_left():
    msg = Twist()
    msg.angular.z = 0.1
    return msg

def follow_the_grapes():
    global regions_
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def turn_right():
    msg = Twist()
    msg.angular.z = 0.1


    

if __name__ == '__main__':
    # as usual, initialise the ROS node with a name
    rospy.init_node('mover')

    Mover()
    # Finally, keep going until we are interrupted.
    rospy.spin()

