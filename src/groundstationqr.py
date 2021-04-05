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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
from pyzbar import pyzbar
#from scipy.interpolate import splprep, splev
#image_topic="/camera/depth/image_raw"
image_topic="/d400/color/image_raw"

class Hole_detector:

    def __init__(self, output=True):     
        #self.image_pub = rospy.Publisher("camera/depth/image_rect_raw",Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic,Image,self.callback)
    def callback(self,data):
        try:       
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        cv2.imshow("image",cv_image)

        # find the barcodes in the image and decode each of the barcodes
        barcodes = pyzbar.decode(cv_image)
        # loop over the detected barcodes
        for barcode in barcodes:
            # extract the bounding box location of the barcode and draw the
            # bounding box surrounding the barcode on the image
            (x, y, w, h) = barcode.rect
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # the barcode data is a bytes object so if we want to draw it on
            # our output image we need to convert it to a string first
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            # draw the barcode data and barcode type on the image
            text = "{} ({})".format(barcodeData, barcodeType)
            cv2.putText(cv_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 255), 2)
            # print the barcode type and data to the terminal
            print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        # show the output image
        cv2.imshow("Image", cv_image)
        cv2.waitKey(0)
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