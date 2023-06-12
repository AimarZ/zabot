#!/usr/bin/env python3
import traceback
from time import sleep

import torch
import supervision as sv
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator
import cv2
import numpy as np
import os
import pickle
import time

import shutil

file_path = "sensor_image.jpg"
    
def drawCross(img, x, y, color, d):
	cv2.line(img, (x+d, y-d), (x-d, y+d), color, 2)
	cv2.line(img, (x-d, y-d), (x+d, y+d), color, 2)

def applyErodeAndDilate(im):
        kernel = np.ones((12,12), np.uint8)
        
        eroded = cv2.erode(im,  kernel, iterations = 1)
        dilated = cv2.dilate(eroded, kernel, iterations = 1)
        return dilated

def findCentroid(img):
	
	img_inv = cv2.bitwise_not(img)

	image_mask = cv2.imread("imageMaskv3.jpg")[:,:,1].astype(np.uint8)
	
	#img_inv = cv2.cvtColor(img_inv, cv2.COLOR_BGR2GRAY)
	img_inv = cv2.bitwise_and(img_inv,image_mask)

	img_segmented = applyErodeAndDilate(img_inv)

	#img_segmented = img_segmented.astype(np.uint8)
	#	img_segmented = cv2.convertTo(img_segmented, CV_32F, 1.0/255.0)
	

	# contours, hierarchy = cv2.findContours(img_segmented, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	#contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours, _ = cv2.findContours(img_segmented, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


	resx = -1
	resy = -1
	numcontours = len(contours)
	print("Objects found: " + str(numcontours))
	if numcontours < 1:
		return resx, resy, img_inv, numcontours, -1   

	color = (255,255,255)
	
	
	cv2.drawContours(img_inv, contours, -1, color, 3)


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
			#cv2.rectangle(img_segmented, (x[i], y[i]), (x[i]+w[i], y[i]+h[i]),  color2, 2, 8, 0 )
		i = i+1

	#Centroid estimate
	if maxsize >= 10:
		M = cv2.moments( biggestContour, False )
		resx = int(M['m10']/M['m00'])
		resy = int(M['m01']/M['m00'])

	#print("Maxsize: ",maxsize)
	#print("Centroid: ", resx, resy)
	# print("%d contours found. Biggest size: %d centroid(%d,%d)"%(len(contours), maxsize, resx, resy))
	return resx,resy, img_segmented, numcontours, maxsize #img_inv

def main():
	
	global file_path, mask_generator

	image_bgr = cv2.imread(file_path)
	print("Sensor image has been read")
	orig = "/home/bee/Desktop/"+file_path
	dest = "/home/bee/Desktop/copy"+file_path
	shutil.copyfile(orig, dest)
	os.remove(file_path)

	image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
	result = mask_generator.generate(image_rgb)

	detections = sv.Detections.from_sam(result)
	for i in range(1):
		mask = detections.mask[i]
		img = mask.astype(np.uint8)  #convert to an unsigned byte
		img*=255
		cv2.imwrite("SAM_output.pgm",img)
		orig = "/home/bee/Desktop/SAM_output.pgm"
		dest = "/home/bee/Desktop/copySAM_output.pgm"
		shutil.copyfile(orig, dest)
	
	resx,resy, img_processed, numc, maxsize = findCentroid(img)

	if numc<1 or maxsize<20:
		label=0 #No piece found
		resx = -1
		resy=-1
	else:
		img_processed = cv2.cvtColor(img_processed, cv2.COLOR_GRAY2RGB)
		drawCross( img_processed, resx, resy, (0,0,255), 5)
		cv2.imwrite("BLOB_output.jpg", img_processed)
		
		cv2.imshow("blob segment",img_processed)
		cv2.setWindowProperty("blob segment", cv2.WND_PROP_TOPMOST, 1)

		cv2.waitKey(3000)
		cv2.destroyAllWindows()

		bgr = image_bgr[resy,resx]

		if bgr[0]>150 and bgr[2]<40:
			label = 2 #Blue
		elif bgr[1]>130:
			label = 3 #Yellow
		else:
			label = 1 #Red 
	
	with open('centroid.pkl', 'wb') as f: 
		pickle.dump([resx,resy,label], f, protocol = 2)
	
	



if __name__ == '__main__':
	
	DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
	MODEL_TYPE = "vit_h"

	sam = sam_model_registry[MODEL_TYPE](checkpoint="sam_vit_h_4b8939.pth")
	sam.to(device=DEVICE)

	mask_generator = SamAutomaticMaskGenerator(sam)

	while True:
		while not os.path.exists(file_path):
			time.sleep(1)
		if os.path.isfile(file_path):
			main()























