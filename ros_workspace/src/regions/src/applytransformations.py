#!/usr/bin/python

'''
applytransformations.py - This script applies a series of transformations
to an input side of images and pickles the data.

Bryant Pong / Micah Corah
CSCI-4962
5/10/15

Last Updated: Bryant Pong: 5/11/15 - 4:51 PM
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
	#return [pickle.load(imgFolder+"/"+fileName, "rb") for fileName in sorted(os.listdir(imgFolder))]
	return [pickle.load(open(imgFolder+"/20150423_152021.dat", "rb"))]

# Main function:
def main():
	
	# Load the images to run transformations on:
	data = loadImages("../../../../data/pickle") 

	# The list containing all of the original data and the transformations:
	finalDataset = []	  

	for dataset in data:
		for img in dataset:
			print("img is: \n" + str(img[0]))
			print("features are: \n" + str(img[1]))

			# Extract the image and feature set from each image:
			image = img[0]
			features = img[1] 

			finalDataset.append( (image, features) )

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
			for angle in xrange(-30, 31, 5):
				rotImg, rotFeatures = tf.rotateCenter(image, features, angle)

				print("rotFeatures: " + str(rotFeatures))

				finalDataset.append( (rotImg, rotFeatures) ) 

				'''
				plt.axis("off")
				plt.imshow(rotImg)		  
				plt.show()
				'''

			# Generate transformations 13 - 14 (Perspective Transformations):
			trans13 = tf.hTrans(image)
			trans14 = tf.hTrans(image)

			finalDataset.append( (trans13, features))
			finalDataset.append( (trans14, features))
			
			# Generate transformations 15 - 16 (Saturation Transformations):
			trans15 = tf.sTran(image, -30)
			trans16 = tf.sTran(image, 30)

			finalDataset.append( (trans15, features) )
			finalDataset.append( (trans16, features) )
	

	# Write the data:
	pickle.dump(finalDataset, open("dataset.p", "wb"))	 			
		
	print("data dump complete")			

# Main function runner:
if __name__ == "__main__":
	main()


