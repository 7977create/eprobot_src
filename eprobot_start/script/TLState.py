#!/usr/bin/python


import os
import time
import cv2, cv_bridge
import rospy
import random
import numpy as np
from enum import Enum
#from detectColor import detectColor
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
#import matplotlib.pyplot as plt
#import matplotlib.image as mpimg


class DETECTLIGHT:
    def __init__(self):
      
        self.color = 0
      
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=20)
        self.image_pub = rospy.Publisher('/image_hsv/rst', Image, queue_size=10)
        self.image_pub_compressed = rospy.Publisher('/image_hsv/rst/compressed', CompressedImage, queue_size=10)
        #self.image_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.image_callback, queue_size=20)
    
    
    
    def detectColor(self,image): # receives an ROI containing a single light    # convert BGR image to HSV

        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

        # min and max HSV values
        red_min = np.array([0,5,150])
        red_max = np.array([8,255,255])
        red_min2 = np.array([175,5,150])
        red_max2 = np.array([180,255,255])

        yellow_min = np.array([20,5,150])
        yellow_max = np.array([30,255,255])

        green_min = np.array([35,5,150])
        green_max = np.array([90,255,255])

        # apply red, yellow, green thresh to image

        red_thresh = cv2.inRange(hsv_img,red_min,red_max)+cv2.inRange(hsv_img,red_min2,red_max2)
        yellow_thresh = cv2.inRange(hsv_img,yellow_min,yellow_max)
        green_thresh = cv2.inRange(hsv_img,green_min,green_max)

        # apply blur to fix noise in thresh

        red_blur = cv2.medianBlur(red_thresh,5)
        yellow_blur = cv2.medianBlur(yellow_thresh,5)
        green_blur = cv2.medianBlur(green_thresh,5)

        # checks which colour thresh has the most white pixels
        red = cv2.countNonZero(red_blur)
        yellow = cv2.countNonZero(yellow_blur)
        green = cv2.countNonZero(green_blur)

        # the state of the light is the one with the greatest number of white pixels
        lightColor = max(red,yellow,green)

        # pixel count must be greater than 60 to be a valid colour state (solid light or arrow)
        # since the ROI is a rectangle that includes a small area around the circle
        # which can be detected as yellow
        if lightColor > 60:
            if lightColor == red:
                return 1
            elif lightColor == yellow:
                return 2
            elif lightColor == green:
                return 3
        else:
            return 0
    
    def imgResize(self,image, height, inter = cv2.INTER_AREA):
        # initialize the dimensions of the image to be resized and grab the image size
        
        dim = None
        (h, w) = image.shape[:2]
        # calculate the ratio of the height and construct the dimensions
        
        r = height / float(h)
        dim = (int(w * r), height)
        # resize the image

        resized = cv2.resize(image, dim, interpolation = inter)
        # return the resized image
        
        return resized

    #def detectState(image, TLType):
    def detectState(self,image):
        
        image = self.imgResize(image, 200)
        (height, width) = image.shape[:2]     
        output = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20,
                                param1=50,param2=30,minRadius=15,maxRadius=30)
        #print(circles)           
        color = 0
        if circles is not None:
            circles = np.uint16(np.around(circles))

            for i in circles[0,:]:
                if i[1] < i[2]:
                    i[1] = i[2]
                roi = image[(i[1]-i[2]):(i[1]+i[2]),(i[0]-i[2]):(i[0]+i[2])]
                color = self.detectColor(roi)
                print('###################')
                print(color)
                cv2.circle(output, (i[0], i[1]), i[2], (0, 0, 255), 3)   # Draw circle
                #cv2.circle(gray, (i[0], i[1]), 2, (0, 255, 0), 5)     # Draw center of circle
        hsv_image_rst_compressed = self.bridge.cv2_to_compressed_imgmsg(output)
        self.image_pub_compressed.publish(hsv_image_rst_compressed)
        return color


    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
        #random.shuffle(image)
        self.color = self.detectState(image)
        #print(self.color)

        
#main function
if __name__=="__main__":
    try:
        rospy.init_node('detect_image',anonymous=True)
        rospy.loginfo('detect image start...')
        
        detectlight = DETECTLIGHT()
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
        print("Shutting down")
        











'''def plot_arrow_result(images):

    for i, image in enumerate(images):
        plt.subplot(1, len(images), i+1)
        lena = mpimg.imread(image)
        label = TLState(detectState(cv2.imread(image),TLType.five_lights.value)).name
        plt.title(label)
        plt.imshow(imgResize(lena, 200))
    plt.show()

arrow_path = ["images/red_greenarrow.png", "images/red_yellowarrow.png"]
random.shuffle(arrow_path)
plot_arrow_result(arrow_path)'''
