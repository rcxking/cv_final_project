#!/usr/bin/python

'''
transformations.py - This module contains functions that perform 
transformations on images.  

Bryant Pong / Micah Corah
CSCI-4962
5/9/15

Last Updated: Bryant Pong: 5/11/15 - 5:01 PM
'''

# Python Imports:
import cv2
import numpy as np
import math
from matplotlib import pyplot as plt

'''
This function rotates an image by a given number of degrees around
the center of the image.  Note that the center of the image is (width / 2, 
height / 2) rather than (0,0), as the origin is by default the upper-left
hand corner of the image.     

Arguments:
img: The image to rotate (represented as a numpy array).
features: The features of img
angle: The angle to rotate (in degrees)   
'''
def rotateCenter(img, features, angle):
	
	# The center of the image:
	centerX = img.shape[1] / 2
	centerY = img.shape[0] / 2   

	# Homogenous matrix that represents all transformations together:
	finalTransform = cv2.getRotationMatrix2D((centerX, centerY), angle, 1)

	#print("finalTransform: " + str(finalTransform))
	#print("finalTransform shape: " + str(finalTransform.shape))

	# Rotate the features - NEED HELP HERE
	featureList = [np.dot(finalTransform, np.array([ [feature[0]], [feature[1]], [0] ])).tolist() for feature in features] 

	return cv2.warpAffine(img, finalTransform, (img.shape[1], img.shape[0])), featureList

'''
This function performs a perspective transformation on an image to simulate
natural camera vibrations made while the robot is moving.   

Arguments:
img: The input to perform a transformation on 
'''
def hTrans(img):
	
	'''
	The following steps are performed on the input image:

	1) Determine the corner points of the image
	2) Add some Gaussian noise to each corner point
	3) Find the perspective matrix given these corner points
	4) Apply this perspective matrix to the input image
	'''

	# The dimensions of the image:
	rows = img.shape[0]
	cols = img.shape[1]

	# Step 1: Find the corner points (in (colX, rowY) format): 
	cornerPts = [0.0,0,cols,0,0,rows,cols,rows]

	# Step 2: Generate some Gaussian Noise for each point:
	gaussNoise = np.random.normal(0, 0.1, 8) * 2000
	noisyPts = [float(cornerPts[i]+gaussNoise[i]) for i in xrange(8)]

	# Step 3: Calculate the perspective matrix from the original image to the transformed image:
	
	# Construct the points from cornerPts and noisyPts:
	actualCorners = np.float32([[cornerPts[i],cornerPts[i+1]] for i in xrange(0, len(cornerPts), 2)])
	actualNoisy = np.float32([[noisyPts[i],noisyPts[i+1]] for i in xrange(0, len(noisyPts), 2)])

	pMat = cv2.getPerspectiveTransform(actualCorners, actualNoisy)

	# Apply the perspective matrix as a perspective transformation to the original image:
	return cv2.warpPerspective(img, pMat, (img.shape[1], img.shape[0])) 

'''
This transformation changes the saturation in an input image to simulate
changing ambient lightning conditions (i.e. shadows or sunlight).

Arguments:
1) img - The input image
2) sat - The saturation to set the image to         
'''
def sTran(img, sat):

	# Convert the image from BGR to HSV:
	imgCopy = np.copy(img)
	hsvImg = cv2.cvtColor(imgCopy, cv2.COLOR_BGR2HSV) 		

	# Change the middle argument in the image to sat:
	hsvImg[:,:,1] += sat

	# Convert the image back to BGR:
	convImg = cv2.cvtColor(hsvImg, cv2.COLOR_HSV2BGR)

	return convImg
