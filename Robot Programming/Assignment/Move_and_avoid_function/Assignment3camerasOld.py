#!/usr/bin/env python
#Heavily based on image project 3 workshop solution and the opencv example
#https://github.com/LCAS/CMP9767M/blob/master/uol_cmp9767m_tutorial/scripts/image_projection_3.py
#https://github.com/LCAS/CMP9767M/blob/master/uol_cmp9767m_tutorial/scripts/opencv_test.py
# Python libs
from pickle import TRUE
import sys, time
import math
from multiprocessing import ProcessError
from turtle import position
# OpenCV
import cv2
from cv2 import blur, Canny, resize, INTER_CUBIC
import numpy as np
# Ros libraries
#sudo apt update
#sudo apt install python-image-geometry
import roslib, rospy, image_geometry, tf

#message filters
import message_filters
# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
class image_projection:
    camera_model = None
    image_depth_ros = None
    
    visualisation = True
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the kinectv2 urdf file
    # (84.1/1920) / (70.0/512)
    object_coordinates = set()

    def __init__(self):

        #front camera------------------------------------------------------------
        self.bridge = CvBridge()
        self.object_location_pub = rospy.Publisher('/thorvald_001/object_location', PoseStamped, queue_size=10)

        front_camera_subs = [
            message_filters.Subscriber('/thorvald_001/kinect2_front_camera/hd/camera_info', CameraInfo),
            message_filters.Subscriber('/thorvald_001/kinect2_front_camera/hd/image_color_rect', Image),
            message_filters.Subscriber('/thorvald_001/kinect2_front_sensor/sd/image_depth_rect', Image),
        ]

        fs = message_filters.ApproximateTimeSynchronizer(front_camera_subs, 1, 0.1, allow_headerless=True)
        fs.registerCallback(self.image_cb)

        right_camera_subs = [
            message_filters.Subscriber('/thorvald_001/kinect2_right_camera/hd/camera_info', CameraInfo),
            message_filters.Subscriber('/thorvald_001/kinect2_right_camera/hd/image_color_rect', Image),
            message_filters.Subscriber('/thorvald_001/kinect2_right_sensor/sd/image_depth_rect', Image),
        ]

        rs = message_filters.ApproximateTimeSynchronizer(right_camera_subs, 1, 0.1, allow_headerless=True)
        rs.registerCallback(self.image_cb)
        
        left_camera_subs = [
            message_filters.Subscriber('/thorvald_001/kinect2_left_camera/hd/camera_info', CameraInfo),
            message_filters.Subscriber('/thorvald_001/kinect2_left_camera/hd/image_color_rect', Image),
            message_filters.Subscriber('/thorvald_001/kinect2_left_sensor/sd/image_depth_rect', Image),
        ]

        ls = message_filters.ApproximateTimeSynchronizer(left_camera_subs, 1, 0.1, allow_headerless=True)
        ls.registerCallback(self.image_cb)

        self.tf_listener = tf.TransformListener()

    def image_cb(self, camera_info_msg, rgb_msg, depth_msg):
        try:
            color2depth_aspect = (84.1/1920) / (70.0/512)
            camera_model = image_geometry.PinholeCameraModel()
            camera_model.fromCameraInfo(camera_info_msg)
            
            # covert images to open_cv
            try:
                image_color = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
                image_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            except CvBridgeError as e:
                print (e)


            # wait for camera_model and depth image to arrive
            if camera_model is None:
                return

            if depth_msg is None:
                return

            # if kernel is too big then the blobs wont be detected    
            kernelOpen=np.ones((10,10))# uses two techniques called dialation and erosion to open and image an filter out noise to increase the accuracy of the mask
            kernelClose=np.ones((25,25))


            # detect a grape in the color image
            image_mask = cv2.inRange(image_color, (100,30,55), (255,255,255))
            
            #convert BGR to HSV
            image_colorHSV= cv2.cvtColor(image_color,cv2.COLOR_BGR2HSV)
            cv2.imshow('HSV',image_colorHSV)
            #morphology open and close to remove noise in the image
            image_maskClose=cv2.morphologyEx(image_mask,cv2.MORPH_CLOSE,kernelClose)
            image_maskOpen=cv2.morphologyEx(image_maskClose,cv2.MORPH_OPEN,kernelOpen)
            
            #conts stores the number of contours that are detected in the image
            image_maskFinal=image_maskOpen
            _,conts,h=cv2.findContours(image_maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            # calculate moments of the binary image
            M = cv2.moments(image_mask)

            if M["m00"] == 0:
                print ('No grapes detected.')
                return
            else:
                print("grapes detected in", camera_info_msg)
            # calculate the y,x centroid
            image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])
            # "map" from color to depth image
            depth_coords = (image_depth.shape[0]/2 + (image_coords[0] - image_color.shape[0]/2)*color2depth_aspect,
                image_depth.shape[1]/2 + (image_coords[1] - image_color.shape[1]/2)*color2depth_aspect)
            # get the depth reading at the centroid location
            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!

            print ('image coords: ', image_coords)
            print ('depth coords: ', depth_coords)
            print ('depth value: ', depth_value)

            # calculate object's 3d location in camera coords
            camera_coords = camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])) #project the image coords (x,y) into 3D ray in camera coords
            camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
            camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth

            print ('camera coords: ', camera_coords)
            bunches=0
            #define a point in camera coordinates
            object_location = PoseStamped()
            object_location.header.frame_id = camera_info_msg.header.frame_id
            object_location.pose.orientation.w = 1.0
            object_location.pose.position.x = camera_coords[0]
            object_location.pose.position.y = camera_coords[1]
            object_location.pose.position.z = camera_coords[2]

            # publish so we can see that in rviz
            self.object_location_pub.publish(object_location)
            

            # print out the coordinates in the map frame
            p_camera = self.tf_listener.transformPose('map', object_location)

            print ('map coords: ', p_camera.pose.position)
            print ('')
            temp = p_camera.pose.position
            self.object_coordinates.add(temp)

            #temp_array = np.array( list(self.object_coordinates) )

            # self.object_coordinates = temp_array.copy()
            bunches =len(self.object_coordinates)

            #Now that i have a map coordinate i save it to a tuple and for each contor created i only count it if the current map coordinate is new, i.e if looking at a new grape.
            cv2.drawContours(image_color,conts,-1,(255,0,0),1)
            for i in range(len(conts)):
                x,y,w,h=cv2.boundingRect(conts[i])
                cv2.rectangle(image_color,(x,y),(x+w,y+h),(0,0,255), 2)# we draw a box around each contour 
                cv2.putText(image_color, str(i+1),(x,y+h),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0))#count the number of contours there are

            #counts number of bounding boxes on screen    
            print(bunches, " bunches of grapes have been detected")
            #print(self.object_coordinates)
            if self.visualisation:
                # draw circles
                cv2.circle(image_color, (int(image_coords[1]), int(image_coords[0])), 10, 255, -1)
                cv2.circle(image_depth, (int(depth_coords[1]), int(depth_coords[0])), 5, 255, -1)

                cv2.circle(image_color, (int(image_coords[0]), int(image_coords[0])), 10, 255, -1)
                cv2.circle(image_depth, (int(depth_coords[0]), int(depth_coords[0])), 5, 255, -1)

                cv2.circle(image_color, (int(image_coords[1]), int(image_coords[1])), 10, 255, -1)
                cv2.circle(image_depth, (int(depth_coords[1]), int(depth_coords[1])), 5, 255, -1)

                cv2.circle(image_color, (int(image_coords[0]), int(image_coords[1])), 10, 255, -1)
                cv2.circle(image_depth, (int(depth_coords[0]), int(depth_coords[1])), 5, 255, -1)

                #resize and adjust for visualisation

                image_color = cv2.resize(image_color, (0,0), fx=0.5, fy=0.5)
                image_depth *= 3.0/10.0 # scale for visualisation (max range 10.0 m)
            # if active_camera == "Front Camera":
            #     front_image_color = image_color
            #     front_image_depth = image_depth
            #     cv2.imshow("front_image depth", front_image_depth)
            #     cv2.imshow("front_image color", front_image_color)
            #     cv2.waitKey(1)
            # if active_camera == "Right Camera":
            #     right_image_color = image_color
            #     right_image_depth = image_depth
            #     cv2.imshow("right_image depth", right_image_depth)
            #     cv2.imshow("right_image color", right_image_color)
            #     cv2.waitKey(1)
            # if active_camera == "Left Camera":
            #     left_image_color = image_color
            #     left_image_depth = image_depth
            #     cv2.imshow("left_image depth", left_image_depth)
            #     cv2.imshow("left_image color", left_image_color)
            #     cv2.waitKey(1)

            cv2.imshow("image_maskClose", image_maskClose)
            cv2.imshow("image_maskOpen",image_maskOpen)
            cv2.imshow("image_mask",image_mask)
            cv2.imshow("cam",image_color)
        except Exception as e:
            print(e)


   #def image_color_callback_master(self,data,frame_id,active_camera):

            # cv2.namedWindow("front_image color")
            # cv2.namedWindow("left_image color")
            # cv2.namedWindow("right_image color")

            # cv2.namedWindow("front_image depth")
            # cv2.namedWindow("left_image depth")
            # cv2.namedWindow("right_image depth")






            



            
          
               



            
    #Created multiple methods for each camera i will be using and renamed the main methods that will be called multiple times 
    
    #Front camera callback methods
    def front_camera_info_callback(self,data):
        self.camera_info_callback_master(data)
        self.front_camera_info_sub.unregister() #Only subscribe once
    def front_image_color_callback(self, data):
        self.image_color_callback_master(data, data.header.frame_id, "Front Camera")#"thorvald_001/kinect2_front_rgb_optical_frame","Front Camera")
    def front_image_depth_callback(self,data):
        self.image_depth_callback_master(data)

    #Right camera callback methods
    def right_camera_info_callback(self,data):
        self.camera_info_callback_master(data)          
        self.right_camera_info_sub.unregister() #Only subscribe once
    def right_image_color_callback(self, data):
        self.image_color_callback_master(data, data.header.frame_id, "Right Camera")#"thorvald_001/kinect2_right_rgb_optical_frame","Right Camera")
    def right_image_depth_callback(self, data):
        self.image_depth_callback_master(data)

    #Left camera callback methods
    def left_camera_info_callback(self, data):
        self.camera_info_callback_master(data)
        self.left_camera_info_sub.unregister() #Only subscribe once
    def left_image_color_callback(self, data):
        self.image_color_callback_master(data,"thorvald_001/kinect2_left_rgb_optical_frame","Left Camera")
    def left_image_depth_callback(self, data):
        self.image_depth_callback_master(data)

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_projection', anonymous=True)
    ic = image_projection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
