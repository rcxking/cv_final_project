#!/usr/bin/python

'''
applytransformations.py - This script applies a series of transformations
to an input side of images and pickles the data.

Bryant Pong / Micah Corah
CSCI-4962
5/10/15

Last Updated: Bryant Pong: 5/14/15 - 5:10 PM
'''

# Python Imports:
import cv2
import numpy as np
from matplotlib import pyplot as plt
import cPickle as pickle # cPickle is faster for Python 2.x
import os # listdir()
import transformations as tf # Custom transformations

'''
This function loads pickled data of images and features
to process.     
'''
def loadImages(imgFolder):

	'''
	data = []

	for fileName in sorted(os.listdir(imgFolder)):
		# Open the next file:
		nextFile = open(imgFolder+"/"+fileName,"rb")
		data.append(pickle.load(nextFile))		
		nextFile.close()
	'''
	
	#return [pickle.load(open(imgFolder+"/"+fileName, "rb")) for fileName in sorted(os.listdir(imgFolder))]
	return [pickle.load(open(imgFolder+"/20150423_152021.dat", "rb"))]
	
	#return data
	#return [pickle.load(open(imgFolder+"/"+fileName, "rb")) for fileName in sorted(os.listdir(imgFolder))]

# Main function:
def transform():
	
	# Load the images to run transformations on:
	data = loadImages("../../../../data/pickle") 
	print("There are: " + str(len(data)) + " datasets to process")

	# Global dataset:
	globalData = []

	datasetNum = 1
	imgNum = 1
	
	for dataset in data:

		print("Now extracting images from dataset: " + str(datasetNum))
		print("There are: " + str(len(dataset)) + " images in this dataset")

		for img in dataset:
	
			print("Now extracting features for image : " + str(imgNum) + " of dataset: " + str(datasetNum))

			#print("img is: \n" + str(img[0]))
			#print("features are: \n" + str(img[1]))

			# Extract the image and feature set from each image:
			image = img[0]
			features = img[1] 
			#print("features type: " + str(type(features)))
			imgWidth = image.shape[1]
			imgHeight = image.shape[0]

			'''
			print("features: " + str(features))
			print("image: ")
			plt.imshow(image)
			plt.show()
			'''

			# Resize the images and features:
			resizedImage = cv2.resize(image, (int(0.5*imgWidth),int(0.5*imgHeight)))
			resizedFeatures = features * 0.5

			globalData.append( (resizedImage, resizedFeatures) )

			'''
			print("resizedFeatures: " + str(resizedFeatures))
			print("resizedImage: ")
			plt.imshow(resizedImage)
			plt.show()
			'''

			'''
			Apply the following transformations to each image:  

			Transformations 1 - 12: Rotate the image from -30 degrees to 30 degrees
			                        in increments of 5 degrees
			Transformations 13 - 14: Generate two perspective transformations with added
			                        random Gaussian noise to stimulate vibrations in the
									robot while traveling
			Transformations 15 - 16: Generate two saturation transformations with 
			                         increasing/decreasing saturation levels to simulate
									 shadows and sunlight   
			'''

			# Generate the rotated images:
			
			rotImg, rotFeatures = tf.rotateCenter(resizedImage, resizedFeatures, -10)
			rotImg2, rotFeatures2 = tf.rotateCenter(resizedImage, resizedFeatures, 10)

			globalData.append( (rotImg, rotFeatures) ) 
			globalData.append( (rotImg2, rotFeatures2) )

			'''
			plt.imshow(rotImg)
			plt.show()

			plt.imshow(rotImg2)
			plt.show()
			'''

			# Generate transformations 13 - 14 (Perspective Transformations):
			trans13, trans13Features = tf.hTrans(resizedImage, resizedFeatures)
			trans14, trans14Features = tf.hTrans(resizedImage, resizedFeatures)

			'''
			plt.imshow(trans13)
			plt.show()

			plt.imshow(trans14)
			plt.show()
			'''

			globalData.append( (trans13, trans13Features))
			globalData.append( (trans14, trans14Features))
			
			# Generate transformations 15 - 16 (Saturation Transformations):
			trans15 = tf.sTran(resizedImage, -50)
			trans16 = tf.sTran(resizedImage, 50)

			globalData.append( (trans15, resizedFeatures) )
			globalData.append( (trans16, resizedFeatures) )

			'''
			plt.imshow(trans15)
			plt.show()

			plt.imshow(trans16)
			plt.show()
			'''

			imgNum += 1

		imgNum = 0

		datasetNum += 1

	#pickle.dump(globalData, open("data.p", "wb"))
	print("data dump complete")			

	return globalData

# Main function runner:
if __name__ == "__main__":
	transform()


