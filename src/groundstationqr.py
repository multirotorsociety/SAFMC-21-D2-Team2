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
from pyzbar import pyzbar
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
    def callback(self,data):
        try:       
            # cv_image = self.bridge.imgmsg_to_cv2(data , desired_encoding='bgr8')
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data , desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)
        # cv_image=cv2.resize(cv_image,(1280,960))
        cv2.imshow("image",cv_image)

        # find the barcodes in the image and decode each of the barcodes
        barcodes = pyzbar.decode(cv_image)
        # loop over the detected barcodes
        cv_image_orig=cv_image
        for barcode in barcodes:
            cv_inst=cv_image_orig
            # extract the bounding box location of the barcode and draw the
            # bounding box surrounding the barcode on the image
            (x, y, w, h) = barcode.rect
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # the barcode data is a bytes object so if we want to draw it on
            # our output image we need to convert it to a string first
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            # draw the barcode data and barcode type on the image
            text = "{} ({})".format(barcodeData, barcodeType)
            cv2.putText(cv_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            cv2.putText(cv_inst, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            # print the barcode type and data to the terminal
            print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
            if not (barcodeData in self.barcodes):
                self.barcodes.append(barcodeData)
                cv2.imwrite(self.image_path+str(len(self.barcodes))+".png", cv_inst)

        
        # show the output image
        cv2.imshow("QR", cv_image)
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