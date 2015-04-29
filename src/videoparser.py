#!/usr/bin/python

'''
videoparser.py - A helper script that extracts image frames from a video via
OpenCV.

Computational Vision Final Project
Bryant Pong/Micah Corah
CSCI-4962
4/24/15

Last Updated: Bryant Pong - 4/25/15 - 6:59 PM
'''

# Python Imports:
import cv2
import numpy as np
import os
import sys

# Main function:
def main(video, datafolder):

	# Check if the data folder to write to already exists and if not create it:
	if not os.path.exists(datafolder):
		print(str(datafolder) + " does not exist!  Creating directory.")
		os.makedirs(datafolder) 
	else:
		print(str(datafolder) + " already exists!")
	
	# Load the video:
	vid = cv2.VideoCapture(video)

	# The image name to write:
	imageName = 1

	while(vid.isOpened()):

		ret, frame = vid.read()

		# Write this next frame to the data folder:
		nextFile = str(datafolder) + "IMG_" + str(imageName) + ".jpg"
		cv2.imwrite(nextFile, frame)

		imageName += 1

		cv2.imshow('frame', frame)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	vid.release()
	cv2.destroyAllWindows()

# Main function runner.  Pass in the path to the video you wish to parse:
if __name__ == '__main__':

	if len(sys.argv) < 3:
		print("Usage: videoparser.py <src video> <dst folder>")  
	else:
		main(sys.argv[1], sys.argv[2]) 
