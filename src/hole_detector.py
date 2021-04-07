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
#from scipy.interpolate import splprep, splev
#image_topic="/camera/depth/image_raw"
image_topic="/d400/depth/image_rect_raw"

class Hole_detector:

    def __init__(self, output=True):     
        #self.image_pub = rospy.Publisher("camera/depth/image_rect_raw",Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic,Image,self.callback)
        self.running_sum=[]
        self.hole_centroids=[]
        self.hole_median_x=-1
        self.hole_median_y=-1
        self.hough_centroids=[]
        self.hough_median_x=-1
        self.hough_median_y=-1
        self.space_centroids=[]
        self.space_median_x=-1
        self.space_median_y=-1
        self.CannyThreshold1=10
        self.CannyThreshold2=20
        self.space=-1
        self.rows=0
        self.cols=0
        self.lasthole=time.time()
        self.lasthough=time.time()
    def unsuscribe(self):
        self.image_sub.unregister()
    def callback(self,data):
        kernel = np.ones((5, 5), np.uint8)
        kernel2 = np.ones((7, 7), np.uint8)
        #print("image")
        try:       
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        if(image_topic=="/d400/depth/image_rect_raw"):
            cv_image=cv_image[:,80:-80]
            (self.rows,self.cols) = cv_image.shape
            cvhue=cv2.merge(((cv_image*255/1000).astype(np.uint8),np.full( (self.rows,self.cols),255, dtype=np.uint8 ),np.full( (self.rows,self.cols),255, dtype=np.uint8 )))
        else:
            cv_image=cv_image/5
            #cv_image=cv2.resize(cv_image, (1280,800),interpolation=cv2.INTER_AREA)
            (self.rows,self.cols) = cv_image.shape
            cvhue=cv2.merge(((cv_image*255).astype(np.uint8),np.full( (self.rows,self.cols),255, dtype=np.uint8 ),np.full( (self.rows,self.cols),255, dtype=np.uint8 )))
        # cv_image=cv_image[180:-180,240:-160]
        cvbgr=cv2.cvtColor(cvhue,cv2.COLOR_HSV2BGR)
        #print(cv_image)
        #cv2.imshow("original",cv_image*10)
        # cv_image = cv2.filter2D(cv_image,-1,kernel2)
        #cv_image = cv2.blur(cv_image,(5,5))
        #cv2.imshow("Image window", cvbgr)


        #cv2.imshow("image",cv_image)
        cv2.waitKey(3)
        cv_image = cv2.morphologyEx(cv_image, cv2.MORPH_ERODE, kernel)
        self.running_sum.append(np.mean(cv_image))
        if len(self.running_sum)>20:
            self.running_sum.pop(0)
        #print(mean(self.running_sum))

        # cv_image=cv2.convertTo(cv_image, CV_8U, 0.00390625)
        if(image_topic=="/d400/depth/image_rect_raw"):
            image_8=np.uint8(cv_image/256)
            image_8=np.uint8(cv_image/128)
        else:
            image_8=np.uint8(cv_image*50)
        #blur

        #cv2.imshow("8bit",image_8)
        #kernel3 = np.ones((5,5),np.float32)/25
        #image_8 = cv2.filter2D(image_8,-1,kernel3)
        image_c=cv2.cvtColor(image_8, cv2.COLOR_GRAY2BGR)
        
        
        
        #detect centre of blank space
        threshold=2000/128
        image_space=image_8[100:-100][:]
        ret2,thresh2 = cv2.threshold(image_space,threshold,255,cv2.THRESH_BINARY)
        cv2.imshow("thresh",thresh2)
        M = cv2.moments(thresh2)
        spaceX=int(M['m10']/M["m00"]) if M["m00"] !=0 else 0
        spaceY=int(M['m01']/M["m00"]) if M["m00"] !=0 else 0
        if M["m00"] !=0:
            # print("a")
            self.space_centroids.append([spaceX*spaceY,spaceX,spaceY])
            if len(self.space_centroids)>5:
                self.space_centroids.pop(0)
            space_centroid_median=np.sort(self.space_centroids,axis=0)[len(self.space_centroids)//2]
            self.space_median_x=space_centroid_median[1]
            self.space_median_y=space_centroid_median[2]+100
            cv2.circle(image_c, (self.space_median_x, self.space_median_y), 7, (0, 255, 0), -1)
            cv2.putText(image_c, "empty space", (spaceX - 20, spaceY + 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (205, 255, 205), 2)
        
        # detect hole

        # image_8=cv2.cvtColor(cv_image,cv2.COLOR_GRAY2BGR)
        #print(image_8.dtype)
        
        hist = cv2.calcHist([image_8],[0],None,[256],[0,256])
        threshold=100
        totalpix=self.rows*self.cols/4
        currentpix=0
        for i in range(len(hist)):
            threshold=i
            currentpix+=hist[i]
            if currentpix>=totalpix:
                threshold+=5
                break
            
        threshold = 2000/128
        ret2,thresh = cv2.threshold(image_8,threshold,255,cv2.THRESH_BINARY)

        thresh=cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        #cv2.imshow("thresh",thresh)


        if(image_topic=="/d400/depth/image_rect_raw"):
            #image_canny=cv2.Canny(image_8, 10, 20)
            image_canny=cv2.Canny(thresh, self.CannyThreshold1, self.CannyThreshold2)
        else:
            
            image_canny=cv2.Canny(thresh, self.CannyThreshold1, self.CannyThreshold2)


        #cv2.imshow("canny",image_canny)
        #cv2.createTrackbar("th1", "canny" , 0, 200, print)
        #cv2.createTrackbar("th2", "canny" , 0, 200, change_threshold2)
        

        ret, th = cv2.threshold(image_canny, 20, 255, 0)

        # --- Find all the contours in the binary image ---
        # contours, hierarchy = cv2.findContours(th, 2, 1)

        th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel)



        #cv2.imshow('th closing', th)
        #contours, hierarchy = cv2.findContours(th, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = cv2.findContours(th, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        cnt = contours
        big_contour = []
        max = 2000
        big_x=-1
        big_y=-1

        for index, i in enumerate(cnt):
            # print("countour")
            area = cv2.contourArea(i)  # --- find the contour having biggest area ---
            # perimeter = cv2.arcLength(i, True)
            # if perimeter == 0:
            #     break
            # circularity = 4 * math.pi * (area / (perimeter * perimeter))

            if (area > max):
                # cv2.putText(image, "center", (cX - 20, cY - 20),
                # cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                perimeter = cv2.arcLength(i, True)
                if perimeter == 0:
                    break
                circularity = 4 * math.pi * (area / (perimeter * perimeter))

                #image_c = cv2.drawContours(image_c, i, -1, (0, 255 if index%2==1 else 0, 255 if index%2==0 else 0),3)
                if 0.7 < circularity < 1.4:
                    # compute the center of the contour
                    M = cv2.moments(i)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(image_c, (cX, cY), 7, (255, 255, 255), -1)
                    #contours_cirles.append(con)
                    max = area
                    big_contour = i
                    big_x=cX
                    big_y=cY
        if big_x!=-1:
        # print(big_contour)
            #print(max)
            image_c = cv2.drawContours(image_c, big_contour, -1, (255, 0, 0), 3)
            self.hole_centroids.append([big_x*big_y,big_x,big_y])
            if len(self.hole_centroids)>5:
                self.hole_centroids.pop(0)
            centroid_median=np.sort(self.hole_centroids,axis=0)[len(self.hole_centroids)//2]
            self.hole_median_x=centroid_median[1]
            self.hole_median_y=centroid_median[2]
            self.lasthole=time.time()
            #print(big_x,big_y,self.hole_median_x,self.hole_median_y)
        if self.hole_median_x!=-1:
            cv2.circle(image_c, (self.hole_median_x, self.hole_median_y), 7, (255, 0, 0), -1)
            cv2.putText(image_c, "center", (self.hole_median_x - 20, self.hole_median_y - 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        ##houghcircles

        image_8_blur = cv2.blur(image_8,(5,5))
        circles = cv2.HoughCircles(image_8_blur,cv2.HOUGH_GRADIENT,1,50,
                            param1=10,param2=70,minRadius=0,maxRadius=0)
        # ensure at least some circles were found
        if circles is not None:
            print("circle found")
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            maxr=[0,0,0]
            for (x, y, r) in circles:
                if r>maxr[2]:
                    maxr=[x,y,r]
            
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(image_c, (maxr[0], maxr[1]), maxr[2], (255, 255, 0), 4)
            cv2.rectangle(image_c, (maxr[0] - 5, maxr[1] - 5), (maxr[0] + 5, maxr[1] + 5), (0, 128, 255), -1)
            self.hough_centroids.append([maxr[0]*maxr[1],maxr[0],maxr[1]])
            if len(self.hough_centroids)>5:
                self.hough_centroids.pop(0)
            centroid_median=np.sort(self.hough_centroids,axis=0)[len(self.hough_centroids)//2]
            self.hough_median_x=centroid_median[1]
            self.hough_median_y=centroid_median[2]
            self.lasthough=time.time()
        cv2.circle(image_c, (self.hough_median_x, self.hough_median_y), 7, (0, 0, 255), -1)
        cv2.putText(image_c, "hough", (self.hough_median_x - 20, self.hough_median_y - 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        #find centre empty space

        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        
        cnt = contours
        big_contour = []
        centreness=9999

        for index, i in enumerate(cnt):
            # print("countour")
            area = cv2.contourArea(i)  # --- find the contour having biggest area ---
            if area < 1000:
                continue
            M = cv2.moments(i)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centre=abs(cX-self.cols/2) + abs(cY-self.rows/2)
            if (centreness > centre):
                big_contour = i
                centreness=centre
            image_c = cv2.drawContours(image_c, i, -1, (0,255,255),3)

        if big_contour != []:
            M = cv2.moments(big_contour)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            if centreness!=9999:
            # print(big_contour)
                #print(max)
                image_c = cv2.drawContours(image_c, big_contour, -1, (255, 255, 0), 3)
                #self.hole_centroids.append([cX*cY,cX,cY])
                #if len(self.hole_centroids)>5:
                #    self.hole_centroids.pop(0)
                #centroid_median=np.sort(self.hole_centroids,axis=0)[len(self.hole_centroids)//2]
                #self.hole_median_x=centroid_median[1]
                #self.hole_median_y=centroid_median[2]
                #self.lasthole=time.time()

        

        # show the output image

        cv2.imshow('final', image_c)
    
    def get_hole(self):
        return self.hole_median_x/self.cols-1/2 if self.hole_median_x!=-1 and self.lasthole > time.time() - 2.5 else None
    def get_hough(self):
        return self.hough_median_x/self.cols-1/2 if self.hough_median_x!=-1 and self.lasthough > time.time() - 2.5  else None
    
    def get_space(self):
        return self.space_median_x/self.cols-1/2



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