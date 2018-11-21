#!/usr/bin/python

from helper_classes.simulator_config import SimulatorConfig

from simulator_helper_functions import *

class Simulator:

    def __init__(self):

        self.sim_config = SimulatorConfig()
        self.loadConfig( self.sim_config )

        self.current_dir = os.path.dirname( os.path.abspath(__file__) )

        self.setupSimulation()

        self.ego_veh_image     = loadImage(self, self.sim_config.ego_veh_image_file )
        self.traffic_veh_image = loadImage(self, self.sim_config.traffic_veh_image_file )

        loadEnvironment(self)

        setupPublishersSubscribers(self)

        rospy.Timer(rospy.Duration(self.sim_config.display_update_duration_s), self.updateDisplay)


    def loadConfig(self, cfg):

        cfg.ego_veh_state_topic        = loadParam("/simulator/ego_veh_state_topic", "/ego_veh_state")
        cfg.traffic_states_topic       = loadParam("/simulator/traffic_states_topic", "/traffic_veh_states")

        cfg.window_width               = loadParam("/simulator/window_width", 1280)
        cfg.window_height              = loadParam("/simulator/window_height", 720)

        cfg.env_yaml_file              = loadParam("/simulator/env_yaml_file", "resources/environments/two_lane_straight_two_cars.yaml")
        cfg.env_bg_image_file          = loadParam("/simulator/env_bg_image_file", "resources/environments/two_lane_straight.png")

        cfg.ego_veh_image_file         = loadParam("/simulator/ego_veh_image_file", "resources/ego_veh_small.png")
        cfg.traffic_veh_image_file     = loadParam("/simulator/traffic_veh_image_file", "resources/traffic_veh_small.png")

        cfg.display_update_duration_s  = loadParam("/simulator/display_update_duration_s", 0.02)

        cfg.pixels_per_meter           = loadParam("/vehicle_description/pixels_per_meter", 14.16)

    def setupSimulation(self):

        pygame.init()
        pygame.display.set_caption("2D Car Simulator")

        print "PyGame window initialized!"

        self.screen = pygame.display.set_mode((self.sim_config.window_width, self.sim_config.window_height))

        self.bg_image = loadImage(self, self.sim_config.env_bg_image_file)
        self.screen.blit(self.bg_image, (0, 0))

    def egoVehStateReceived(self, state):

        self.ego_veh.pose.x = state.pose.x;
        self.ego_veh.pose.y = state.pose.y;

        self.ego_veh.pose.theta = state.pose.theta;

    def updateDisplay(self, event):

        checkPyGameQuit()

        self.screen.blit(self.bg_image, (0, 0))
        rotateAndBlitImage(self.screen, self.ego_veh_image, self.ego_veh.pose.getPosition(), self.ego_veh.pose.theta )

        pygame.display.update()

        if self.ego_veh.pose.x > 1280:
            rospy.signal_shutdown("Ego vehicle went out of bounds")
