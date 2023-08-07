#!/usr/bin/python3

import os
import rospy
import pygame
import yaml
import math

from helper_classes.vehicle import Vehicle
from helper_classes.road_info import RoadInfo
from helper_classes.pose import Pose

from simulator_msgs.msg import EgoVehicle, TrafficVehicles

def loadEnvironment(sim_obj):

    # Load road related information from the yaml file
    with open(sim_obj.sim_config.env_yaml_file, "r") as env_file:
        sim_obj.env_data = yaml.safe_load(env_file)

    sim_obj.ego_veh = setupEgoVehicle(sim_obj)

    setupTrafficVehicles(sim_obj)

    # Upload scene information to ros parameter server for other packages to use
    rospy.set_param("sim_scene_data", sim_obj.env_data)

    sim_obj.road_info = loadRoadInfo(sim_obj)

def loadRoadInfo(sim_obj):

    temp_env_data = sim_obj.env_data

    road_info = RoadInfo()
    road_info.num_lanes = len(temp_env_data['lanes'])

    for lane_number in range(road_info.num_lanes):

        lanes = temp_env_data['lanes'][lane_number]

        lane_point_list = []
        for lane_point in lanes['lane_points']:

            lane_point_pose = Pose(lane_point['x'], lane_point['y'], lane_point['heading'])
            lane_point_list.append(lane_point_pose)

        road_info.lanes[lanes['lane_id']] = lane_point_list

    return road_info

def setupEgoVehicle(sim_obj):
    ego_veh_id      = 0
    ego_veh_pos     = (sim_obj.env_data['ego_veh_pose']['x'], sim_obj.env_data['ego_veh_pose']['y'])
    ego_veh_heading = sim_obj.env_data['ego_veh_pose']['heading']

    ego_veh = Vehicle(veh_id=ego_veh_id, veh_init_pos=ego_veh_pos, veh_init_heading=ego_veh_heading)
    ego_veh.max_vel = loadParam("/vehicle_description/max_velocity_mps", 17.8816)
    ego_veh.max_accel = loadParam("/vehicle_description/max_acceleration_mps2", 4.0)
    ego_veh.max_steering_angle = loadParam("/vehicle_description/max_steering_angle_degree", 40.0)

    return ego_veh

def setupTrafficVehicles(sim_obj):

    for vehicle in sim_obj.env_data['traffic_vehicles']:

        veh_id = vehicle['vehicle_id']
        veh_pos = (vehicle['vehicle_pose']['x'], vehicle['vehicle_pose']['y'])
        veh_heading = vehicle['vehicle_pose']['heading']

        traffic_vehicle = Vehicle(veh_id=veh_id, veh_init_pos=veh_pos, veh_init_heading=veh_heading)

        traffic_vehicle.vel = vehicle['initial_velocity']

        traffic_vehicle.max_vel = loadParam("/vehicle_description/max_velocity_mps", 17.8816)
        traffic_vehicle.max_accel = loadParam("/vehicle_description/max_acceleration_mps2", 4.0)
        traffic_vehicle.max_steering_angle = loadParam("/vehicle_description/max_steering_angle_degree", 40.0)

        sim_obj.traffic.append(traffic_vehicle)

def setupPublishersSubscribers(sim_obj):
    sim_obj.traffic_states_pub = rospy.Publisher(sim_obj.sim_config.traffic_states_topic, TrafficVehicles, queue_size=1)
    sim_obj.ego_veh_state_sub = rospy.Subscriber(sim_obj.sim_config.ego_veh_state_topic, EgoVehicle, sim_obj.egoVehStateReceived)

def loadImage(sim_obj, filename, new_dims=None):

    image_path = os.path.join(sim_obj.current_dir, filename)
    image = pygame.image.load(image_path).convert_alpha()
    
    if new_dims is not None:
        image = pygame.transform.scale(image, new_dims)

    return image

def loadParam(param_name, default):
    return rospy.get_param(param_name, default)

def checkPyGameQuit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()

def rotateAndBlitImage(surface, image, center_pos, heading_angle):
    rotated_image = pygame.transform.rotozoom(image, -heading_angle, 1)
    rotated_image_rect = rotated_image.get_rect()

    rotated_image_rect.center = center_pos[0], center_pos[1]

    surface.blit(rotated_image, rotated_image_rect)

# @brief Converts position to simulator coordinate system
def convertPosToSimCoordinates(sim_obj, world_position):

    # X and Y coordinates converted from meters to pixel values for display purposes
    x_pixels = world_position[0] * sim_obj.sim_config.pixels_per_meter
    y_pixels = world_position[1] * sim_obj.sim_config.pixels_per_meter

    # Invert Y coordinates to match simulator coordinate system
    y_pixels = sim_obj.sim_config.window_height - y_pixels

    return x_pixels, y_pixels

# @brief Returns center pos using vehicle origin coordinates as input
def convertOriginToVehicleCenter(sim_obj, origin_position):
    center_position_x = origin_position[0] + (sim_obj.ego_veh.wheel_base/2)*math.cos(sim_obj.ego_veh.pose.heading)
    center_position_y = origin_position[1] + (sim_obj.ego_veh.wheel_base/2)*math.sin(sim_obj.ego_veh.pose.heading)

    return center_position_x, center_position_y

# @brief convert angle to simulator coordinates degrees
def convertToSimDegrees(heading):
    return math.degrees(-heading)
