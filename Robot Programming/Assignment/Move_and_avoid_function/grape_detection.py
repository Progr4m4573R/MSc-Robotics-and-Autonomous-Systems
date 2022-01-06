#!/usr/bin/env python3
#!/usr/bin/env python2
#!/usr/bin/env python
#Object detection for grape
#mport numpy
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
import mover

class Camera:
    
    def process_image(camera_feed): 
        try:
            bridge = CvBridge
        except Exception as err:
            print (err)






   



    def __init__(self):

        #Create a camera subscriber for input
        self.s = rospy.Subscriber('/thorvald_001/kinetic2_front_camera/hd/image_color_rect',Image, self.process_image)
    
        #Create a camera publisher for Image processing
        self.bridge = CvBridge()
        rospy.init_node('grape_detection')
        rospy.loginfo('grape detection node active')















if __name__ == '__main__':
    #Prevent code from breaking with exception
    try:
        rospy.init_node('Camera')
        Camera()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass