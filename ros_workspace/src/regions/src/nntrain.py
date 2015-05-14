#!/usr/bin/python

'''
nntrain.py - This script trains a neural network given a set of images
to help identify regions of interest.

Bryant Pong / Micah Corah      
CSCI-4962
5/4/15

Last Updated: Bryant Pong: 5/14/15 - 1:52 AM
'''

# Python Imports:
import numpy as np
import cv2
from sklearn.neural_network import BernoulliRBM
from sklearn.linear_model import LogisticRegression # A logistic classifier
from sklearn.pipeline import Pipeline
import slidingwindow as sw
import cPickle as pickle # cPickle is faster for Python 2.X 
import os # Open function
import random
import applytransformations as atf

# Training function:
def train():
	
	# The neural network object:
	neuralNetwork = BernoulliRBM()   

	# The logistic regression classifier:
	logisticRegression = LogisticRegression(C=1.0)

	'''
	This is a scikit Pipeline object that represents the order of operations
	to apply for the classifier.
	
	Our pipeline resembles the following:
	Step 1 - Neural Network
	Step 2 - Logistic Regression classifier		    
	'''
	classifier = Pipeline([("NN", neuralNetwork), ("logistic", logisticRegression)])

	# Load the training data:
	print("Now loading training data")
	trainingData = atf.transform()
	print("Loaded training data") 
		
	print("Now extracting data and running sliding window algorithm")
	'''
	Next iterate through each image in the training data and run the sliding 
	window algorithm.
	'''

	# These variables hold the subimages and their labels:
	data = []
	labels = []		 

	'''
	The size of the sliding window.  Images are of dimension length 640
	and height 360.
	''' 
	swWidth = 80
	swHeight = 120

	curNum = 1

	print("There is: " + str(len(trainingData)) + " images to process")
	for nextData in trainingData:

		print("Now processing image: " + str(curNum) + " out of: " + str(len(trainingData)) + " images")
		#print("nextImage: " + str(nextData[0]))
		#print("nextFeatures: " + str(nextData[1]))

		# Now run the sliding window algorithm:
		windows = sw.slidingWindow(nextData[0], swWidth, swHeight)  

		# These lists hold the good windows and good features:
		goodWindows = []
		goodFeatures = []	
		
		'''
		Determine the labels of the images. 
		Our images are 640 x 360.  The sliding windows will be of size 80 x 120.
		This results in 24 windows:
		'''  
		nextLabels = np.zeros((3, 8))
		#print("nextLabels: " + str(nextLabels))

		# For each feature, determine the appropriate window:
		for feature in nextData[1]:

			#print("feature: " + str(feature))
			# Calculate which window to be in:
			winRow = feature[1] / swHeight
			winCol = feature[0] / swWidth	
			
			#print("winRow: " + str(winRow))
			#print("winCol: " + str(winCol))	 		  	      

			# Check for negative coordinates:
			if winRow >= 0 and winCol >= 0 and winRow < 3 and winCol < 8:
				nextLabels[int(winRow),int(winCol)] = 1

		# Now filter out any windows with too much black:
		for i in xrange(len(windows)):
			if (windows[i].shape[0] * windows[i].shape[1]) - cv2.countNonZero(cv2.cvtColor(windows[i], cv2.COLOR_BGR2GRAY)) < 0.1 * windows[i].shape[0] * windows[i].shape[1]: 
				goodWindows.append(windows[i].flatten().tolist())
				goodFeatures.append(nextLabels.flatten().tolist()[i])

		#print("goodWindows len: " + str(len(goodWindows)))
		#print("goodFeatures len: " + str(len(goodFeatures)))
		# Extend the data and labels arrays:
		data.extend(goodWindows)
		labels.extend(goodFeatures)	 

		curNum += 1

	print("Done extracting data and running sliding window algorithm")		

	# Train the neural network:
	print("Now training classifier:")
	print("data length: " + str(len(data)))
	print("labels length: " + str(len(labels)))
	#print("data.shape: " + str(np.array(data).shape))
	#print("labels.shape: " + str(np.array(labels).shape))
	print("Finally training classifier:")
	#print("data[0]: " + str(data[0]))
	'''
	print("len(data[0]: " + str(len(data[0])))
	for i in xrange(0, len(labels), 3):
		print("Fitting on data set: " + str(i))
		x = np.array([data[i], data[i+1], data[i+2]])
		y = np.array([labels[i], labels[i+1], labels[i+2]])
		print("x.shape: " + str(x.shape))
		print("y.shape: " + str(y.shape))
		classifier.fit(x, y)
	'''

	random.shuffle(data)
	random.shuffle(labels)

	#classifier.fit(np.array(data)[:len(data)/2], np.array(labels)[:len(labels)/2])
	classifier.fit(np.array(data[:5000]), np.array(labels[:5000]))
	print("Done training classifier:")

	print("Now writing classifier")
	pickle.dump(classifier, open("classifier.p", "wb"))
	print("Complete!")

			
# Training Runner
if __name__ == "__main__":
	train()
