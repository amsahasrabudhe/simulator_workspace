#!/usr/bin/python

import os
import rospy
import pygame

from simulator_config import SimulatorConfig

class Simulator:

    def __init__(self):

        self.sim_config = SimulatorConfig()
        self.loadConfig( self.sim_config )

        self.current_dir = os.path.dirname(os.path.abspath(__file__))

        self.setupSimulation()

        self.auto_car_image    = self.loadImage( self.sim_config.autonomous_car_image_file )
        self.traffic_car_image = self.loadImage( self.sim_config.traffic_car_image_file )

        

        rospy.Timer(rospy.Duration(0.02), self.updateDisplay)


    def loadConfig(self, cfg):

        def loadParam(param_name, default):
            return rospy.get_param("/simulator/"+param_name, default)

        cfg.window_width               = loadParam("window_width", 1280)
        cfg.window_height              = loadParam("window_height", 720)

        cfg.env_yaml_file              = loadParam("env_yaml_file", "resources/environments/two_lane_straight_two_cars.yaml")
        cfg.env_bg_image_file          = loadParam("env_bg_image_file", "resources/environments/two_lane_straight.png")

        cfg.autonomous_car_image_file  = loadParam("autonomous_car_image_file", "resources/autonomous_car_small.png")
        cfg.traffic_car_image_file     = loadParam("traffic_car_image_file", "resources/traffic_car_small.png")

        cfg.display_update_duration_s  = loadParam("display_update_duration_s", 0.02)


    def setupSimulation(self):

        pygame.init()
        pygame.display.set_caption("2D Car Simulator")

        self.screen = pygame.display.set_mode((self.sim_config.window_width, self.sim_config.window_height))

        self.bg_image = self.loadImage(self.sim_config.env_bg_image_file)
        self.screen.blit(self.bg_image, (0, 0))


    def loadImage(self, filename):

        image_path = os.path.join(self.current_dir, filename)
        return pygame.image.load(image_path).convert_alpha()


    def updateDisplay(self, event):

        self.screen.blit(self.bg_image, (0, 0))
        self.screen.blit( pygame.transform.rotozoom(self.auto_car_image, 0, 1), (self.car_posX,360) )
        self.car_posX += 5

        pygame.display.update()
