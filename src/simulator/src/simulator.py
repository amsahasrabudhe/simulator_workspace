#!/usr/bin/python

from helper_classes.simulator_config import SimulatorConfig

from simulator_helper_functions import *

class Simulator:

    def __init__(self):

        self.sim_config = SimulatorConfig()
        self.loadConfig( self.sim_config )

        self.current_dir = os.path.dirname(os.path.abspath(__file__))

        self.setupSimulation()

        self.auto_car_image    = loadImage(self, self.sim_config.autonomous_car_image_file )
        self.traffic_car_image = loadImage(self, self.sim_config.traffic_car_image_file )

        loadEnvironment(self)

        rospy.Timer(rospy.Duration(self.sim_config.display_update_duration_s), self.updateDisplay)


    def loadConfig(self, cfg):

        cfg.window_width               = loadParam("/simulator/window_width", 1280)
        cfg.window_height              = loadParam("/simulator/window_height", 720)

        cfg.env_yaml_file              = loadParam("/simulator/env_yaml_file", "resources/environments/two_lane_straight_two_cars.yaml")
        cfg.env_bg_image_file          = loadParam("/simulator/env_bg_image_file", "resources/environments/two_lane_straight.png")

        cfg.autonomous_car_image_file  = loadParam("/simulator/autonomous_car_image_file", "resources/autonomous_car_small.png")
        cfg.traffic_car_image_file     = loadParam("/simulator/traffic_car_image_file", "resources/traffic_car_small.png")

        cfg.display_update_duration_s  = loadParam("/simulator/display_update_duration_s", 0.02)

        cfg.pixels_per_meter           = loadParam("/vehicle_description/pixels_per_meter", 14.16)

    def setupSimulation(self):

        pygame.init()
        pygame.display.set_caption("2D Car Simulator")

        self.screen = pygame.display.set_mode((self.sim_config.window_width, self.sim_config.window_height))

        self.bg_image = loadImage(self, self.sim_config.env_bg_image_file)
        self.screen.blit(self.bg_image, (0, 0))

    def updateDisplay(self, event):

        checkPyGameQuit()

        self.screen.blit(self.bg_image, (0, 0))
        rotateAndBlitImage(surface=self.screen, image=self.auto_car_image,
                           center_pos=self.auto_car.pose.getPosition(), heading_angle=self.auto_car.pose.theta )

        self.auto_car.pose.x += 5

        pygame.display.update()
