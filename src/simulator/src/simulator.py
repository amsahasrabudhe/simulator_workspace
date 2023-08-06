#!/usr/bin/python3

from helper_classes.simulator_config import SimulatorConfig
from traffic_generation_functions import *
from simulator_helper_functions import *

from simulator_msgs.msg import Vehicle

import os

class Simulator:

    def __init__(self):

        # Variable to hold ego vehicle related information
        self.ego_veh = Vehicle()

        # List containing traffic vehicles
        self.traffic = []

        # Variable to hold simulator configuration related information
        self.sim_config = SimulatorConfig()
        self.loadConfig(self.sim_config)

        self.current_dir = os.path.dirname(os.path.abspath(__file__))

        if self.sim_config.render:
            self.setupSimulation()

            scaled_image_dims = (int(Vehicle().length*self.sim_config.pixels_per_meter), int(Vehicle().width*self.sim_config.pixels_per_meter))
            self.ego_veh_image     = loadImage(self, self.sim_config.ego_veh_image_file, scaled_image_dims)
            self.traffic_veh_image = loadImage(self, self.sim_config.traffic_veh_image_file, scaled_image_dims)

        loadEnvironment(self)

        setupPublishersSubscribers(self)

        # Last update time for traffic vehicle calculations
        self.last_update_time = rospy.get_time()


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

        cfg.render                     = loadParam("/simulator/render", False)

        cfg.pixels_per_meter           = loadParam("/vehicle_description/pixels_per_meter", 4.26)

    def setupSimulation(self):

        pygame.init()
        pygame.display.set_caption("2D Car Simulator")

        print("PyGame window initialized!")

        self.screen = pygame.display.set_mode((self.sim_config.window_width, self.sim_config.window_height))

        self.bg_image = loadImage(self, self.sim_config.env_bg_image_file)
        self.screen.blit(self.bg_image, (0, 0))

    def egoVehStateReceived(self, ego):

        self.ego_veh.pose.x = ego.vehicle.pose.x
        self.ego_veh.pose.y = ego.vehicle.pose.y
        self.ego_veh.pose.heading = ego.vehicle.pose.theta

        self.ego_veh.steering = ego.vehicle.steering
        self.ego_veh.vel = ego.vehicle.vel
        self.ego_veh.accel = ego.vehicle.accel

        self.ego_veh.length = ego.vehicle.length
        self.ego_veh.width = ego.vehicle.width

        # Display updated position of ego vehicle on screen along with updates to traffic vehicle position
        self.updateDisplay()

    def publishTrafficInformation(self):

        ros_traffic = TrafficVehicles()
        ros_traffic.header.stamp = rospy.Time.now()

        for vehicle in self.traffic:

            ros_vehicle = Vehicle()

            ros_vehicle.pose.x = vehicle.pose.x
            ros_vehicle.pose.y = vehicle.pose.y
            ros_vehicle.pose.theta = vehicle.pose.heading

            ros_vehicle.vel = vehicle.vel
            ros_vehicle.accel = vehicle.accel
            ros_vehicle.steering = vehicle.steering

            ros_vehicle.length = vehicle.length
            ros_vehicle.width = vehicle.width

            ros_traffic.traffic.append(ros_vehicle)

        self.traffic_states_pub.publish(ros_traffic)

    def displayInformationText(self):

        text_font = pygame.font.Font('freesansbold.ttf', 20)
        text_surface = text_font.render("EGO VEHICLE SPEED : "+str(self.ego_veh.vel*2.23694)[:5]+" MPH", True, (0, 0, 255))

        text_rect = text_surface.get_rect()
        text_rect.center = (1100, 25)

        self.screen.blit(text_surface, text_rect)

    def displayEgoVehicle(self):

        # Display Ego vehicle using data received on rostopic
        ego_center_pos = convertOriginToVehicleCenter(self, self.ego_veh.pose.getPosition())
        ego_sim_pos = convertPosToSimCoordinates(self, ego_center_pos)
        rotateAndBlitImage(self.screen, self.ego_veh_image, ego_sim_pos, convertToSimDegrees(self.ego_veh.pose.heading))

    def displayTrafficVehicles(self):

        for veh in self.traffic:
            veh_pos = convertPosToSimCoordinates(self, veh.pose.getPosition())
            rotateAndBlitImage(self.screen, self.traffic_veh_image, veh_pos, convertToSimDegrees(veh.pose.heading))

    def displayVehicleOrigin(self):

        origin_pos = convertPosToSimCoordinates(self, self.ego_veh.pose.getPosition())
        self.screen.fill((0, 255, 0), (origin_pos, (2, 2)))

    def updateDisplay(self):

        if self.sim_config.render:
            checkPyGameQuit()

        updateTrafficVehiclesPositions(self, rospy.get_time())
        self.publishTrafficInformation()

        if self.sim_config.render:
            self.screen.blit(self.bg_image, (0, 0))

            self.displayEgoVehicle()
            self.displayTrafficVehicles()
            self.displayInformationText()

            self.displayVehicleOrigin()

            pygame.display.update()

        if self.ego_veh.pose.x * self.sim_config.pixels_per_meter > 1280:
            pygame.quit()
            rospy.signal_shutdown("Car went out of bounds")
