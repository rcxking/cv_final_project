#!/usr/bin/python

'''
video.py - This ROS node extracts frames from a webcam and returns the 
individual frames.

Bryant Pong / Micah Corah
CSCI-4962
4/30/15 

Last Updated: Bryant Pong: 5/1/15 - 3:28 PM
'''

# Python Imports:
import cv2
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from regionvideo.msg import Ints
import numpy as np
from matplotlib import pyplot as plt

# This function constantly reads in images from the webcam:
def video_extract():

	# Create a ROS publisher for images:
	frames = rospy.Publisher("frames", numpy_msg(Ints))

	# Initialize this code as a ROS node:
	rospy.init_node("framenode")

	# The OpenCV Camera Object:
	camera = cv2.VideoCapture(0)

	while not rospy.is_shutdown():
		
		# Extract frames and publish them:
		ret, frame = camera.read()
		print("frame: \n" + str(frame.flatten()))
		#a = np.array([1, 2, 3, 4], dtype=np.uint8)
		#frames.publish(np.array(frame.flatten()))
		a = frame.flatten().tolist()
		print("a: " + str(a))
		#frames.publish(np.array(frame.flatten().tolist(), dtype=np.uint8))
		frames.publish(np.array([1, 2, 3, 4], dtype=np.uint8))
		
		

# Main function runner:
if __name__ == "__main__":
	try:
		video_extract()
	except rospy.ROSInterruptException:
		pass
