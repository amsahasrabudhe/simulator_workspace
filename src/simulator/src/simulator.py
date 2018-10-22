#!/usr/bin/python

import os
import rospy
import time

import pygame

WINDOW_WIDTH  = 1280
WINDOW_HEIGHT = 720

class Simulator:

    def __init__(self):

        pygame.init()
        pygame.display.set_caption("2D Car Simulator")

        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))

        self.screen.fill((255,255,255))

        current_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(current_dir, "resources/car_small.png")

        self.car_image = pygame.image.load(image_path)

        self.car_posX = 1280

        rospy.Timer(rospy.Duration(0.02), self.updateDisplay)


    def updateDisplay(self, event):

        self.screen.fill((255, 255, 255))
        self.screen.blit( pygame.transform.rotate(self.car_image, 0), (self.car_posX,360) )
        self.car_posX -= 5

        pygame.display.update()
