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
import math

'''
returns point nearest to two skew lines
'''
def nearestPoint(p1, d1, p2, d2):
  # least squares solution to nearest points on each line
  b = np.array([[2*np.dot(p2,d1) - 2*np.dot(p1,d1)],
                [2*np.dot(p1,d2) - 2*np.dot(p2,d2)]])
  a = np.array([[2*np.dot(d1,d1), -2*np.dot(d2,d1)],
                [-2*np.dot(d2,d1), 2*np.dot(d2,d2)]])

  # coefficients for nearest points
  # p+s*d
  ss = np.linalg.solve(a, b)

  # nearest points
  n1 = p1 + ss[0] * d1
  n2 = p2 + ss[1] * d2

  # nearest point is average
  return (n1 + n2) / 2

'''
angle between to vectors
'''
def innerAngle(d1, d2):
  d1p = d1 / np.linalg.norm(d1)
  d2p = d2 / np.linalg.norm(d2)
  return math.acos(np.dot(d1p, d2p))


'''
returns distance between two skew lines
see
http://2000clicks.com/mathhelp/GeometryPointsAndLines3D.aspx
'''
def lineDistance(p1, d1, p2, d2):
  n = np.cross(d1, d2)
  np.dot(p1 - p2, n) / np.linalg.norm(n)

def inBounds(p, bounds):
  return p[0] > bounds[0] and p[0] < bounds[1] \
         and p[1] > bounds[2] and p[1] < bounds[3] \
         and p[2] > bounds[4] and p[2] < bounds[5]
         
def cluster(points, 
            directions,
            threshold_distance = 0.1,
            bounds = (0,1,0,1,0,1),
            plot_result = True,
            ground_truth = None):
  print "Clustering"
  num_point = points.shape[1]

  correspondence_points = []
  for ii in range(num_point):
    for jj in range(ii+1, num_point):
      p1 = points[:,ii]
      d1 = directions[:,ii]
      p2 = points[:,jj]
      d2 = directions[:,jj]
      # interested in nearby skew lines
      if lineDistance(p1, d1, p2, d2) < threshold_distance \
          and innerAngle(d1, d2) > np.pi / 180:
        nearest = nearestPoint(p1, d1, p2, d2)
        # throw out points that are outside of the region of interest
        if inBounds(nearest, bounds):
          #print "Found correspondence:", nearest
          correspondence_points.append(nearest)
  correspondence_points = np.array(correspondence_points).T
  print correspondence_points.shape

  if plot_result:
    # plot figure for correspondence
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(correspondence_points[0,:], correspondence_points[1,:], correspondence_points[2,:], c='b', marker='x')
    if not ground_truth is None:
      ax.scatter(ground_truth[0,:], ground_truth[1,:], ground_truth[2,:], c='r', marker='o')
    plt.show()

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
    sigma_observation = 0.03,
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


  number_false_positive = int(false_positives_per_observation * \
      number_observation * number_points_of_interest)

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
  #plotLines(points, directions)
  cluster(points, directions, ground_truth = points_of_interest,
      threshold_distance = sigma_observation)

if __name__ == "__main__":
  testCluster()
