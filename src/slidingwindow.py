#!/usr/bin/python

'''
slidingwindow.py - This module loads an image and returns
a list of images resulting from the sliding window technique.

Bryant Pong / Micah Corah
CSCI-4962
Undergraduate
4/29/15

Last Updated: Bryant Pong: 4/29/15 - 7:47 PM
'''

# Python imports:
import cv2
import numpy as np
from matplotlib import pyplot as plt

'''
This function performs the sliding window algorithm and
returns a list of all the images generated.

winLen, winHgt are in pixels.  
'''
def slidingWindow(img, winLen, winHgt): 

	# Size of the image:
	imgLen = img.shape[1]
	imgHgt = img.shape[0]

	# Ensure that this image can be evenly divided by winLen, winHgt:
	if winLen == 0 or winHgt == 0 or imgLen % winLen != 0 or imgHgt % winHgt != 0:
		print("Error window parameters do not divide img evenly!")
		return []	
	
	# Determine the number of window blocks to cover:
	numBlkLen = int(imgLen / winLen)
	numBlkHgt = int(imgHgt / winHgt)

	print("You can create: " + str(numBlkLen) + " blocks horizontally x " + \
	      str(numBlkHgt) + " blocks vertically")
		
	imgs = []
	# Extract the blocks:
	for m in xrange(numBlkHgt):
		for n in xrange(numBlkLen):
			nextCornerRow = m*winHgt
			nextCornerCol = n*winLen

			#print("next corner at (" + str(nextCornerCol) + "," + str(nextCornerRow) + ")")

			imgs.append(img[nextCornerRow:nextCornerRow+winHgt, nextCornerCol:nextCornerCol+winLen])

	return imgs
