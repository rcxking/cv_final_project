#!/usr/bin/python

'''
main.py - Main entry point for regions of interest extraction.

Bryant Pong / Micah Corah
CSCI-4962
4/30/15

Last Updated: Bryant Pong: 5/4/15 - 4:30 PM
'''

# Python Imports:
import cv2
import rospy
import numpy as np
from sklearn.neural_network import BernoulliRBM
from std_msgs.msg import String
from matplotlib import pyplot as plt
import cPickle as pickle

# Custom libraries:
import neuralnetwork as nn
import points_of_interest as poi
import slidingwindow as sw

# Main function:
def main():

	# Intialize ROS:
	rospy.init_node("region_extractor")

	'''
	Create a ROS publisher that publishes the regions of interest:    
	'''

	# TO-DO Change the type of message to publish from String to regions: 
	regionPub = rospy.Publisher('regionext', String, queue_size=10) 

	# Create the cv2 object to get frames from the webcam:
	camera = cv2.VideoCapture(0)	

	# Load the neural network:

	while not rospy.is_shutdown():

		# frame is in BGR color space:
		ret, frame = camera.read()

		# DEBUG ONLY - Print out the size of the frame:
		# Laptop camera returns images of dimension (640 x 480)
		#print("frame dimensions: " + str(frame.shape))

		# Execute the sliding window algorithm on this frame:
		slidingFrames = sw.slidingWindow(frame, 40, 40)    			    

		#print("40 x 40 frames to examine: " + str(slidingFrames))
		#print("Number of 40x40 frames to examine: " + str(len(slidingFrames)))

		regionPub.publish("hello")

		break

	camera.release()
	cv2.destroyAllWindows()


# Main function runner:
if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
