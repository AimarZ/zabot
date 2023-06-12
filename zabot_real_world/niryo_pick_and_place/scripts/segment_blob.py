#!/usr/bin/env python
import rospy
import numpy as np
import traceback
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from time import sleep
from niryo_pick_and_place.srv import BlobSegmentService, BlobSegmentServiceResponse

#Yellow [113 194 178]
#Red [ 28  39 144]
#Blue [194 118   0]


class SegmentBlob:
    def __init__(self):

        #self.green_params = [70,100,0,112,160,50]
        self.blue_params = [60,0,0,220,194,49]
        #self.yellow_params = [120,200,170,150,229,210]
        self.yellow_params = [70,175,0,140,246,221]
        self.red_params = [10,20,130,115,120,235]
    
        self.orig_img = None
        self.params = [self.red_params, self.blue_params, self.yellow_params]

        self.bridge = CvBridge()
        seg_srv = rospy.Service('segment_blob_srv', BlobSegmentService, self.segmentBlobSrv)
        print("Ready to segment blob!")
    
    def setImage(self, ros_img):
        try:
            img = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
            # mask = np.zeros(img.shape,np.uint8)
            # mask[0:-1,65:225] = img[0:-1,65:225] 
            # mask[0:350,225:615] = img[0:350,225:615]
            self.orig_img = img   
        except CvBridgeError as exc:
            print(traceback.format.exc())

    def setParams(self, lowB, lowG, lowR, highB, highG, highR):
        self.params = [lowB, lowG, lowR, highB, highG, highR]
      
    def applyErodeAndDilate(self, im):
        kernel = np.ones((2,2), np.uint8)
        
        eroded = cv2.erode(im,  kernel, iterations = 1)
        dilated = cv2.dilate(eroded, kernel, iterations = 1)
        return dilated

    def segmentBlobSrv(self, req):

        self.setImage(req.image)

        #self.setParams(*self.red_params)

        resx, resy, img, numc, maxsize = self.findCentroid()

        if numc<1 or maxsize<20:
            label=0 #No piece found
            cv2.imshow("Blob segmentation", self.orig_img)
            cv2.startWindowThread()
        else:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            self.drawCross( img, resx, resy, (0,0,255), 5)
            cv2.imshow("Blob segmentation", img)
            cv2.startWindowThread()

            bgr = self.orig_img[resy,resx]

            if bgr[0]>150 and bgr[2]<40:
                label = 2 #Blue
            elif bgr[1]>130:
                label = 3 #Yellow
            else:
                label = 1 #Red 
        
        response = BlobSegmentServiceResponse()
        response.resx = resx
        response.resy = resy
        response.label = label
        rospy.loginfo("Detected blob: Centroid(%d,%d) Label:%d", response.resx, response.resy, response.label)

        return response
    
    def drawCross(self, img, x, y, color, d):
        cv2.line(img, (x+d, y-d), (x-d, y+d), color, 2)
        cv2.line(img, (x-d, y-d), (x+d, y+d), color, 2)

    def findCentroid(self):
        
        src = self.orig_img.copy()

        img_segmented = np.zeros(src.shape[0:2],np.uint8)

        self.params = [self.blue_params, self.red_params, self.yellow_params]

        for i in range(len(self.params)):
            params = self.params[i]

            #params = [0,1,1,255,50,50]
            lowRGB = np.array(params[0:3])
            highRGB = np.array(params[3:])

            aux = cv2.inRange(src, lowRGB, highRGB)

            # # Apply erode and dilate
            aux = self.applyErodeAndDilate(aux)
            img_segmented = cv2.bitwise_or(img_segmented,aux)
        
        image_mask = cv2.imread("/home/bee/Desktop/imageMaskv4.jpg")[:,:,1].astype(np.uint8)
        img_segmented = cv2.bitwise_and(img_segmented,image_mask)

        # Find contours
        # Depending upon cv2 version!!
        # contours, hierarchy = cv2.findContours(img_segmented, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        im, contours, _ = cv2.findContours(img_segmented, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        resx = -1
        resy = -1
        numcontours = len(contours)
        if numcontours < 1:
            return resx, resy, src, numcontours, -1   
    
        color = (255,255,255)
        
        
        cv2.drawContours(src, contours, -1, color, 3)


        #Approximate contours to polygons + get bounding rects and circles
        contours_poly = []
        center = []
        radius = []
        biggestContour = -1
        maxsize = 0
        boundRect = []
        x = [-1 for i in range(numcontours)]
        y = [-1 for i in range(numcontours)]
        w = [-1 for i in range(numcontours)]
        h = [-1 for i in range(numcontours)]
        for i in range(0, numcontours):
            contours_poly.append(cv2.approxPolyDP(contours[i], 3, True ))
            x[i], y[i], w[i], h[i] = cv2.boundingRect(contours_poly[i])
            c, r = cv2.minEnclosingCircle(contours_poly[i])
            center.append(c)
            radius.append(r)
        i = 0
        maxindex = -1
        maxsize = 0
        color2 = (255, 0, 0)
        for contour in contours:
            # if contour.size > 10:
            if contour.size > maxsize:
                maxsize = contour.size
                maxindex = i
                biggestContour = contour
                cv2.rectangle(img_segmented, (x[i], y[i]), (x[i]+w[i], y[i]+h[i]),  color2, 2, 8, 0 )
            i = i+1
    
        #Centroid estimate
        if maxsize >= 10:
            M = cv2.moments( biggestContour, False )
            resx = int(M['m10']/M['m00'])
            resy = int(M['m01']/M['m00'])
    
        #print(maxsize)
        # print("%d contours found. Biggest size: %d centroid(%d,%d)"%(len(contours), maxsize, resx, resy))
        return resx,resy, img_segmented, numcontours, maxsize #src

if __name__ == '__main__':
    rospy.init_node("segment_blob")
    segment_blob = SegmentBlob()
    rospy.spin()























