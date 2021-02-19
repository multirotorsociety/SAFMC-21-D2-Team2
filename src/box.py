#!/usr/bin/env python3

import roslib
roslib.load_manifest('SAFMC-21-D2-Team2')
import sys
import rospy
from statistics import mean
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class BoxCV:

    def __init__(self, output=False):     
        #self.image_pub = rospy.Publisher("camera/depth/image_rect_raw",Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/d400/depth/image_rect_raw", Image, self.image_cb)
        #self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.image_cb)
        self.running_sum = []
        self.theta = np.array([0, 0])
        self.output=output

    def image_cb(self, data):
        #print("image")
        try:       
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        # cv_image=cv_image[:,80:]
        cv_image=cv_image[200:-200,560:-400]
        (rows,cols) = cv_image.shape
        if self.output:
            cv2.imshow("image",cv_image/18)
            cv2.waitKey(3)

            self.running_sum.append(np.mean(cv_image))
            if len(self.running_sum)>20:
                self.running_sum.pop(0)
            print("mean:", mean(self.running_sum))

        y_mean = np.mean(cv_image, axis=0)
        x = np.array([[1, i/cols] for i in range(cols)]).reshape(cols, 2)
        self.theta = np.matmul(np.matmul(np.linalg.inv(np.matmul(np.transpose(x), x)), np.transpose(x)), y_mean)
        
        if self.output:
            print(self.theta)
        
    def get_theta(self):
        return self.theta
    
    def get_mean(self):
        return mean(self.running_sum)


def main(args):
    print(sys.version)
    boxCV = BoxCV(output=True)
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)