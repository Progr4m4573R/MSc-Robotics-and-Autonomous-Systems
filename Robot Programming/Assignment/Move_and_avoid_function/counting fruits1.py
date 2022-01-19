import cv2
import numpy as np
from numpy import mean, array
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from cv2 import namedWindow, cvtColor, imshow, inRange
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey, COLOR_BGR2HSV
from cv2 import blur, Canny, resize, INTER_CUBIC

class count_fruits:

    def __init__(self): 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_front_camera/hd/image_color_rect",Image, self.process_image)

    def process_image(self, camera):

        # if kernel is too big then the blobs wont be detected

        self.kernelOpen=np.ones((15,15))# uses two techniques called dialation and erosion to open and image an filter out noise to increase the accuracy of the mask
        self.kernelClose=np.ones((20,20))

        img = self.bridge.imgmsg_to_cv2(camera, "bgr8")
        img = resize(img, None, fx=0.6, fy=0.6, interpolation = INTER_CUBIC)
        #img=cv2.resize(img,(340,220))

        lowerBound=np.array([100,30,20])
        upperBound=np.array([255,256,256])
        font = cv2.FONT_HERSHEY_SIMPLEX

        #convert BGR to HSV
        imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        cv2.imshow('HSV',imgHSV)
        # create the Mask
        mask=cv2.inRange(imgHSV,lowerBound,upperBound)
        
        #morphology
        self.maskClose=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,self.kernelOpen)

        self.maskOpen=cv2.morphologyEx(self.maskClose,cv2.MORPH_OPEN,self.kernelClose)
        #self.maskClose=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,self.kernelClose)

        #conts stores the number of contours that are detected in the image
        maskFinal=self.maskOpen
        _,conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        cv2.drawContours(img,conts,-1,(255,0,0),1)
        for i in range(len(conts)):
            x,y,w,h=cv2.boundingRect(conts[i])
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255), 2)# we draw a box around each contour 
            cv2.putText(img, str(i+1),(x,y+h),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0))#count the number of contours there are
            
        cv2.imshow("maskClose", self.maskClose)
        cv2.imshow("maskOpen",self.maskOpen)
        cv2.imshow("mask",mask)
        cv2.imshow("cam",img)
        if cv2.waitKey(10) &0xFF ==ord('q'):
                    cv2.cap.release()
                    cv2.destroyAllWindows()
                
if __name__ =='__main__':
    rospy.init_node('count_fruits')
    cf = count_fruits()
    rospy.spin()