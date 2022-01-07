#!/usr/bin/env python3
#!/usr/bin/env python2
#!/usr/bin/env python
#Object detection for grape
import rospy
from cv2 import namedWindow, cvtColor, imshow, inRange
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey, COLOR_BGR2HSV
from cv2 import blur, Canny, resize, INTER_CUBIC
from numpy import mean, array
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class robot_image_processor:

    def __init__(self):
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_front_camera/hd/image_color_rect",Image, self.process_image)

    def process_image(self, camera_feed): 
        try:#enable code to continue if there is an error during processing pipeline
        
            namedWindow("Image window")
            namedWindow("masked")
            #namedWindow("canny")
            original_image = self.bridge.imgmsg_to_cv2(camera_feed, "bgr8")
            original_image = resize(original_image, None, fx=0.2, fy=0.2, interpolation = INTER_CUBIC)
            #experimenting---------------------------------------------------
            hsv = cvtColor(original_image, COLOR_BGR2HSV)#converts that the robot sees to hsv
            
            lower_green = array([0, 150, 150])# detect green for leaves
            upper_green = array([255, 255, 255])#this too
            lower_purple = array([75,5,10])# detectes purple for grapes
            upper_purple = array([255,255,255])#this too
            hsv_green_threshold = inRange(hsv, lower_green, upper_green)
            hsv_purple_threshold = inRange(hsv,lower_purple, upper_purple)
            green_mask = hsv_green_threshold
            #experimenting----------------------------------------------------
            mask = inRange(hsv, lower_purple, upper_purple)
            imshow("masked", mask)
            gray_img = cvtColor(hsv, COLOR_BGR2GRAY)
            #img3 = Canny(gray_img, 10, 200)
            #imshow("canny", img3)

            imshow("Image window", hsv)
            waitKey(1)




        except Exception as err:
            print (err)
        #if processing fails, show robot view 

        
if __name__ == '__main__':
    #Prevent code from breaking with exception
    try:
        #startwindow thread
        rospy.init_node('robot_image_processor')
        ric = robot_image_processor()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Code error detected")