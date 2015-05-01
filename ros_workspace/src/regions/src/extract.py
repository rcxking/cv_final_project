#!/usr/bin/python

'''
main.py - Main entry point for regions of interest extraction.

Bryant Pong / Micah Corah
CSCI-4962
4/30/15

Last Updated: Bryant Pong: 4/30/15 - 5:48 PM 
'''

# Python Imports:
import cv2
import rospy
import numpy as np
from regions.srv import Image 
from matplotlib import pyplot as plt

'''
This callback handles receiving an image and extracting regions of 
interest from the image.      
'''
def extractCallback(req):
	return True

# Main function:
def main():

	# Intialize ROS:
	rospy.init_node("region_extractor")

	'''
	Create a ROS service to take in an input image and extract the
	regions of interest:
	'''
	regionSrv = rospy.Service('regionext', Image, extractCallback) 

	print("Ready to accept next image")

	rospy.spin()

# Main function runner:
if __name__ == "__main__":
	main()
