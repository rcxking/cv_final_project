#!/usr/bin/python

'''
cluster_util.py 
math for line clustering

Bryant Pong / Micah Corah      
CSCI-4962
'''

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plotLines(points, directions):
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

  #points
  ax.scatter(points[0,:], points[1,:], points[2,:], c='b', marker='x')

  #lines
  for i in range(points.shape[1]):
    point = points[:,i]
    direction = directions[:,i]
    norm_d = direction / np.linalg.norm(direction)
    ends = np.column_stack([(point - norm_d), (point + norm_d)])
    ax.plot(ends[0,:], ends[1,:], ends[2,:])


  #plt.axis([0,1,0,1,0,1])
  plt.show()

'''
Test line clustering algorithm
Considering lines contained in the unit cube
1. generate points of interest
2. generate normally distributed observations and directions around points of
interest
3. generate large number of false positives uniformly in unit cube
4. run clustering on observations and false positives
'''
def testCluster(
    number_points_of_interest = 4,
    number_observation = 10,
    sigma_observation = 0.05,
    false_positives_per_observation = 2
    ):

  points_of_interest = np.random.ranf((3,number_points_of_interest))
  print "Points of interest"
  print points_of_interest

  observation_points = np.hstack(
      [points_of_interest + sigma_observation * np.random.standard_normal(points_of_interest.shape)
        for i in range(number_observation)])

  observation_directions = np.hstack(
      [np.random.standard_normal(points_of_interest.shape)
        for i in range(number_observation)])


  number_false_positive = false_positives_per_observation * number_observation * number_points_of_interest

  false_positive_points = np.hstack(
      [np.random.ranf((3,1))
        for i in range(number_observation)])

  false_positive_directions = np.hstack(
      [np.random.standard_normal((3,1))
        for i in range(number_observation)])

  points = np.hstack([observation_points, false_positive_points])
  directions = np.hstack([observation_directions, false_positive_directions])
  print points.shape
  print directions.shape
  plotLines(points, directions)

if __name__ == "__main__":
  testCluster()
