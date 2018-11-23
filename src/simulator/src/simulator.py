#!/usr/bin/python

from helper_classes.simulator_config import SimulatorConfig

from simulator_helper_functions import *


class Simulator:

    # Variable to hold simulator configuration related information
    sim_config = None

    # Variable to hold simulation scene related data
    env_data = None

    # Variable to hold ego vehicle related information
    ego_veh = None

    # List containing traffic vehicles
    traffic = []

    def __init__(self):

        self.sim_config = SimulatorConfig()
        self.loadConfig(self.sim_config)

        self.current_dir = os.path.dirname(os.path.abspath(__file__))

        self.setupSimulation()

        self.ego_veh_image     = loadImage(self, self.sim_config.ego_veh_image_file)
        self.traffic_veh_image = loadImage(self, self.sim_config.traffic_veh_image_file)

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

        cfg.pixels_per_meter           = loadParam("/vehicle_description/pixels_per_meter", 14.2259)

    def setupSimulation(self):

        pygame.init()
        pygame.display.set_caption("2D Car Simulator")

        print "PyGame window initialized!"

        self.screen = pygame.display.set_mode((self.sim_config.window_width, self.sim_config.window_height))

        self.bg_image = loadImage(self, self.sim_config.env_bg_image_file)
        self.screen.blit(self.bg_image, (0, 0))

    def egoVehStateReceived(self, ego):

        self.ego_veh.pose.x = ego.vehicle.pose.x;
        self.ego_veh.pose.y = ego.vehicle.pose.y;
        self.ego_veh.pose.theta = ego.vehicle.pose.theta;

        self.ego_veh.steering = ego.vehicle.steering
        self.ego_veh.vel = ego.vehicle.vel
        self.ego_veh.accel = ego.vehicle.accel

        self.ego_veh.length = ego.vehicle.length
        self.ego_veh.width = ego.vehicle.width

    def displayInformationText(self):

        text_font = pygame.font.Font('freesansbold.ttf', 20)
        text_surface = text_font.render("EGO VEHICLE SPEED : "+str(self.ego_veh.vel*2.23694)[:5]+" MPH", True, (0, 0, 255))

        text_rect = text_surface.get_rect()
        text_rect.center = (1100, 25)

        self.screen.blit(text_surface, text_rect)

    def updateDisplay(self, event):

        checkPyGameQuit()

        self.screen.blit(self.bg_image, (0, 0))

        # Display Ego vehicle using data received on rostopic
        ego_sim_pos = convertPosToSimCoordinates(self, self.ego_veh.pose.getPosition())
        rotateAndBlitImage(self.screen, self.ego_veh_image, ego_sim_pos, convertThetaToSimDegrees(self.ego_veh.pose.theta) )

        self.displayInformationText()

        pygame.display.update()

        if self.ego_veh.pose.x * self.sim_config.pixels_per_meter > 1280:
            pygame.quit()
