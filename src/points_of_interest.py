#!/usr/bin/python

import pygame
from pygame.locals import *
import numpy as np
import sys
import cv2

pygame.init()

def points_of_interest(cv_image):
  image = pygame.image.frombuffer(cv_image, (cv_image.shape[1], cv_image.shape[0]), 'RGB')

  size = image.get_size()
  screen = pygame.display.set_mode(size)
  points = np.zeros((0,2), dtype='uint8')

  running = True
  pressed = False
  clock = pygame.time.Clock()
  while running:
    screen.fill((255,255,255))
    screen.blit(image, (0,0))
    pygame.display.flip()
    for event in pygame.event.get():
      if event.type == QUIT:
        running = False
      if event.type == KEYDOWN:
        running = False
      if event.type == MOUSEBUTTONDOWN:
        pos = pygame.mouse.get_pos()
        print "Point:", pos
        points = np.vstack((points, np.array(list(pos), dtype='uint8')), )
    clock.tick(30)
  print "Points:", points
  return points


if __name__ == "__main__":
  file_name = sys.argv[1]
  image = cv2.imread(file_name)#, cv2.LOAD_IMAGE_COLOR)
  image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
  points = points_of_interest(image)
