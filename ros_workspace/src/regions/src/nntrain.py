#!/usr/bin/python

'''
nntrain.py - This script trains a neural network given a set of images
to help identify regions of interest.

Bryant Pong / Micah Corah      
CSCI-4962
5/4/15

Last Updated: Bryant Pong: 5/14/15 - 4:56 PM
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

		# For each feature, determine the appropriate window:
		for feature in nextData[1]:

			# Calculate which window to be in:
			winRow = feature[1] / swHeight
			winCol = feature[0] / swWidth	
			
			# Check for negative coordinates:
			if winRow >= 0 and winCol >= 0 and winRow < 3 and winCol < 8:
				nextLabels[int(winRow),int(winCol)] = 1

		# Now filter out any windows with too much black:
		for i in xrange(len(windows)):
			if (windows[i].shape[0] * windows[i].shape[1]) - cv2.countNonZero(cv2.cvtColor(windows[i], cv2.COLOR_BGR2GRAY)) < 0.1 * windows[i].shape[0] * windows[i].shape[1]: 
				goodWindows.append(windows[i].flatten().tolist())
				goodFeatures.append(nextLabels.flatten().tolist()[i])

		# Extend the data and labels arrays:
		data.extend(goodWindows)
		labels.extend(goodFeatures)	 

		curNum += 1

	print("Done extracting data and running sliding window algorithm")		


	negData = [data[i] for i in xrange(len(data)) if labels[i] == 0]
	posData = [data[i] for i in xrange(len(data)) if labels[i] == 1]

	print("negData: " + str(len(negData)))
	print("posData: " + str(len(posData)))

	finalData = []
	finalLabels = []

	finalData.extend(posData)
	finalLabels.extend([1 for i in xrange(len(posData))])

	finalData.extend(negData[:300])
	#finalLabels.extend([0 for i in xrange(len(posData))])		
	finalLabels.extend([0 for i in xrange(len(negData[:300]))])

	# Train the neural network:
	print("Now training classifier:")
	print("Finally training classifier:")

	classifier.fit(np.array(finalData), np.array(finalLabels))
	print("Done training classifier:")

	print("Now writing classifier")
	pickle.dump(classifier, open("classifier.p", "wb"))
	print("Complete!")

			
# Training Runner
if __name__ == "__main__":
	train()
