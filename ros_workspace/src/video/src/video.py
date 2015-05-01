#!/usr/bin/python

'''
video.py - This ROS node extracts frames from a webcam and returns the 
individual frames.

Bryant Pong / Micah Corah
CSCI-4962
4/30/15 

Last Updated: Bryant Pong: 4/30/15 - 6:18 PM    
'''

# Python Imports:
import cv2
import rospy
from std_msgs.msg import String
import numpy as np
from matplotlib import pyplot as plt

# This function constantly reads in images from the webcam:
def video_extract():

	# Create a ROS publisher for images:
	frames = rospy.Publisher("frames",    

# Main function runner:
if __name__ == "__main__":
	try:
		video_extract()
	except rospy.ROSInterruptException:
		pass
