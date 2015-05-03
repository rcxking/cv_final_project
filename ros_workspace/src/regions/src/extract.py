#!/usr/bin/python

'''
main.py - Main entry point for regions of interest extraction.

Bryant Pong / Micah Corah
CSCI-4962
4/30/15

Last Updated: Bryant Pong: 5/3/15 - 5:08 PM
'''

# Python Imports:
import cv2
import rospy
import numpy as np
from regions.srv import Image 
from std_msgs.msg import String
from matplotlib import pyplot as plt

# Main function:
def main():

	# Intialize ROS:
	rospy.init_node("region_extractor")

	'''
	Create a ROS publisher that publishes   
	'''
	regionPub = rospy.Publisher('regionext', String, queue_size=10) 

	# Create the cv2 object to get frames from the webcam:
	camera = cv2.VideoCapture(0)	

	while not rospy.is_shutdown():
		ret, frame = camera.read()

		regionPub.publish("hello")

	camera.release()
	cv2.destroyAllWindows()


# Main function runner:
if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
