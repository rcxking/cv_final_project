#!/usr/bin/python

'''
extractnode.py - Main entry point for regions of interest extraction.

Bryant Pong / Micah Corah
CSCI-4962
4/30/15

Last Updated: Bryant Pong: 5/15/15 - 1:25 AM
'''

# Python Imports:
import cv2
import rospy
import numpy as np
from sklearn.neural_network import BernoulliRBM
from std_msgs.msg import String
from matplotlib import pyplot as plt
import cPickle as pickle
import os
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

	# Load the neural network and its classifier:
	classifier = pickle.load(open("classifier.p", "rb"))

	while not rospy.is_shutdown():

		# frame is in BGR color space:
		#ret, frame = camera.read()

		frame = cv2.imread("/home/bryant/img1.jpg")
		frame = frame[:,:,::-1]

		# Resize the images such that the images are of dimension (640x360):
		resizedFrame = cv2.resize(frame, (640, 360)) 

		# Execute the sliding window algorithm on this frame:
		slidingFrames = sw.slidingWindow(resizedFrame, 80, 120)    			    

		# Run the classifier prediction algorithm:
		for nextFrame in slidingFrames:

			#plt.imshow(nextFrame)
			#plt.show()	

			nextPredict = classifier.predict(nextFrame.flatten().tolist())

			print("nextPredict: " + str(nextPredict))

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
