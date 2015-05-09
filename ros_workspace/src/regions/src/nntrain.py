#!/usr/bin/python

'''
nntrain.py - This script trains a neural network given a set of images
to help identify regions of interest.

Bryant Pong / Micah Corah      
CSCI-4962
5/4/15

Last Updated: Bryant Pong: 5/9/15 - 4:11 PM
'''

# Python Imports:
import numpy as np
import cv2
from sklearn.neural_network import BernoulliRBM
from sklearn.linear_model import LogisticRegression # A logistic classifier
from sklearn.pipeline import Pipeline
import cPickle as pickle # cPickle is faster for Python 2.X 
import os # listdir() function

'''
This function helps generate transformations  
'''

'''
This function loads Pickle data containing images and the marked regions
of interest:         
'''
def loadData(imgFolder):

	# Uncomment when running real neural network training:
	#return [pickle.load(open(imgFolder+"/"+str(i), "rb")) for i in sorted(os.listdir(imgFolder))]
	return [pickle.load(open(imgFolder+"/20150423_152339.dat", "rb"))] 

# Training function:
def train():
	
	# The neural network object:
	neuralNetwork = BernoulliRBM()   

	# The logistic regression classifier:
	logisticRegression = LogisticRegression()

	'''
	This is a scikit Pipeline object that represents the order of operations
	to apply for the classifier.
	
	Our pipeline resembles the following:
	Step 1 - Neural Network
	Step 2 - Logistic Regression classifier		    
	'''
	classifier = Pipeline([("NN", neuralNetwork), ("logistic", logisticRegression)])

	# Load the training data:
	trainingData = loadData("../../../../data/pickle")
	
	# Next iterate through each image in the dataset and perform transformations on the image:
	for i in xrange(len(trainingData)):
		nextTrainingSet = trainingData[i]

		for j in xrange(len(nextTrainingSet)):
			nextImage = nextTrainingSet[j]

			nextImageData = nextImage[0]
			nextImageFeatures = nextImage[1]
			
			print("nextImageData: " + str(nextImageData))
			print("nextImageFeatures: " + str(nextImageFeatures))				
			'''
			Next let's apply a series of transformations on this image and
			its features. 

			The following image transformations will be made on the images:
			1)  	   	
			'''
					
			
# Training Runner
if __name__ == "__main__":
	train()
