#!/usr/bin/env python3

import roslib
roslib.load_manifest('SAFMC-21-D2-Team2')
import sys
import rospy
import math
from statistics import mean
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import time
import datetime 
#from scipy.interpolate import splprep, splev
#image_topic="/camera/depth/image_raw"
image_topic="/d400/color/image_raw/compressed"

class Hole_detector:

    def __init__(self, output=True):     
        #self.image_pub = rospy.Publisher("camera/depth/image_rect_raw",Image)
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber(image_topic,Image,self.callback)
        self.image_sub = rospy.Subscriber(image_topic,CompressedImage,self.callback)
        self.barcodes=[]
        # Image path
        self.image_path = '/home/pootis/Pictures/SAFMC/'
        today = datetime.datetime.now()
        date_time = today.strftime("%m%d%Y%H%M%S")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        width  = 640
        height = 480
        self.out = cv2.VideoWriter('/home/pootis/output'+date_time+'.avi', fourcc, 20.0, (width,  height))

    def callback(self,data):
        try:       
            # cv_image = self.bridge.imgmsg_to_cv2(data , desired_encoding='bgr8')
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data , desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)
        # cv_image=cv2.resize(cv_image,(1280,960))
        cv2.imshow("image",cv_image)

        #adaptive threshold
        cv_image_orig=cv_image
        cv_image=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # blur = cv2.GaussianBlur(cv_image,(5,5),0)
        blur=cv_image
        ret2,th2 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        th3 = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,7,2)
        #cv2.imshow("blur",blur)
        #cv2.imshow("gauss",th3)
        cv2.imshow("otsu",th2)
        
        kernel=cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))

        closing = cv2.morphologyEx(th2, cv2.MORPH_CLOSE, kernel,iterations=2)
        cv2.imshow("close",closing)

        # Find contours and filter for QR code
        cnts = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            print(approx)
            x,y,w,h = cv2.boundingRect(approx)
            area = cv2.contourArea(c)
            ar = w / float(h)
            if len(approx) == 4 and 100< area < 700 and (ar > .85 and ar < 1.3):
                cv2.rectangle(cv_image_orig, (x, y), (x + w, y + h), (36,255,12), 3)
                ROI = cv_image_orig[y:y+h, x:x+w]
                print(area)

        cv2.imshow("final",cv_image_orig)
        self.out.write(cv_image_orig)

        cv2.waitKey(1)
    def get_space(self):
        return "a"



def main(args):
    print(sys.version)
    ic = Hole_detector()
    rospy.init_node('hole_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)