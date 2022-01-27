#!/usr/bin/env python
from glob import glob
from matplotlib.pyplot import imshow, twinx
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
class Mover:
    object_location = None
    
    def __init__(self):
        #robot_front_camera.image_projection()
        #robot_left_camera.image_projection()
        robot_right_camera.image_projection()

        self.publisher = rospy.Publisher(
            '/thorvald_001/teleop_joy/cmd_vel',
            Twist, queue_size=1)
        rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.laser_callback)
        self.pose_pub = rospy.Publisher(
            '/nearest_obstacle',
            PoseStamped,queue_size=1
        )
        self.listener = TransformListener()

        #Library Used to convert from ROS images to OpenCV
        self.bridge = cv_bridge.CvBridge()

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

        min_dist = min(data.ranges)

        t = self.twist

        # If anything is closer than 4 metres anywhere in the
        # scan, we turn away
        if min_dist < 2:
            t.linear.x = -0.2
            t.angular.z = 2.0
        else:  # if all obstacles are far away, let's keep 
            # moving forward at 0.8 m/s
            t.linear.x = 0.8

        self.publisher.publish(t)

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
        
        # if kernel is too big then the blobs wont be detected

        self.kernelOpen=np.ones((10,10))# uses two techniques called dialation and erosion to open and image an filter out noise to increase the accuracy of the mask
        self.kernelClose=np.ones((25,25))
        image = self.bridge.imgmsg_to_cv2(camera_info_msg,desired_encoding='bgr8')
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        
        # detect a grape in the color image
        image_mask = cv2.inRange(hsv, (100,30,55), (255,255,255))

         #morphology
        self.maskClose=cv2.morphologyEx(image_mask,cv2.MORPH_CLOSE,self.kernelClose)

        self.maskOpen=cv2.morphologyEx(self.maskClose,cv2.MORPH_OPEN,self.kernelOpen)
       
        #conts stores the number of contours that are detected in the image
        maskFinal=self.maskOpen
        _,conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        cv2.drawContours(image,conts,-1,(255,0,0),1)
        for i in range(len(conts)):
            x,y,w,h=cv2.boundingRect(conts[i])
            cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,255), 2)# we draw a box around each contour 
            cv2.putText(image, str(i+1),(x,y+h),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0))#count the number of contours there are
            
        #counts number of bounding boxes on screen    
        print(len(conts), " bunches of grapes have been detected")
        h, w, d = image.shape
        gM = cv2.moments(image_mask)
        
        if gM['m00'] > 0:
            print("grape detected!")
            cx = int(gM['m10']/gM['m00'])
            cy = int(gM['m01']/gM['m00'])
            cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1)

            err = cx - w/2
            self.twist.linear.x = 0.5
            self.twist.angular.z = -float(err) / 1000
            print (t.angular.z)
            print("moving... via moments to grape")
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

if __name__ == '__main__':
    # as usual, initialise the ROS node with a name
    rospy.init_node('mover')

    Mover()
    # Finally, keep going until we are interrupted.
    rospy.spin()

