#!/usr/bin/python

import os
import rospy
import pygame
import yaml

from helper_classes.vehicle import Vehicle
from helper_classes.road_info import RoadInfo
from helper_classes.pose import Pose

from simulator_msgs.msg import VehState

def loadEnvironment(sim_obj):

    # Load road related information from the yaml file
    with open(sim_obj.sim_config.env_yaml_file, "r") as env_file:
        sim_obj.env_data = yaml.load(env_file)

    sim_obj.ego_veh = setupEgoVehicle(sim_obj)

    # Upload scene information to ros parameter server for other packages to use
    rospy.set_param("sim_scene_data", sim_obj.env_data)

    sim_obj.road_info = loadRoadInfo(sim_obj)

    print sim_obj.road_info


def loadRoadInfo(sim_obj):

    temp_env_data = sim_obj.env_data

    road_info = RoadInfo()
    road_info.num_lanes = temp_env_data['num_lanes']

    for lane_number in range(road_info.num_lanes):

        lane_info = temp_env_data['lane_info'][lane_number]

        lane_point_list = []
        for lane_point in lane_info['lane_points']:

            lane_point_pose = Pose(lane_point['x'], lane_point['y'], lane_point['theta'])
            lane_point_list.append(lane_point_pose)

        road_info.lanes[ lane_info['lane_id'] ] = lane_point_list

    return road_info

def setupEgoVehicle(sim_obj):

    ego_veh_id      = 0
    ego_veh_pos     = ( sim_obj.env_data['ego_veh_pose']['x'], sim_obj.env_data['ego_veh_pose']['y'] )
    ego_veh_heading = sim_obj.env_data['ego_veh_pose']['heading']

    ego_veh = Vehicle(veh_id=ego_veh_id, veh_init_pos=ego_veh_pos, veh_init_theta=ego_veh_heading)
    ego_veh.max_vel = loadParam("/vehicle_description/max_velocity_mps", 17.8816)
    ego_veh.max_accel = loadParam("/vehicle_description/max_acceleration_mps2", 4.0)
    ego_veh.max_steering_angle = loadParam("/vehicle_description/max_steering_angle_degree", 40.0)

    return ego_veh

def setupPublishersSubscribers(sim_obj):

    sim_obj.traffic_states_pub = rospy.Publisher(sim_obj.sim_config.traffic_states_topic, VehState, 1)
    sim_obj.ego_veh_state_sub = rospy.Subscriber(sim_obj.sim_config.ego_veh_state_topic, VehState, sim_obj.egoVehStateReceived)

def loadImage(sim_obj, filename):

    image_path = os.path.join(sim_obj.current_dir, filename)
    return pygame.image.load(image_path).convert_alpha()


def loadParam(param_name, default):
    return rospy.get_param(param_name, default)


def checkPyGameQuit():

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            rospy.signal_shutdown("QUIT requested")


def rotateAndBlitImage(surface, image, center_pos, heading_angle):

    rotated_image = pygame.transform.rotozoom(image, -heading_angle, 1)
    rotated_image_rect = rotated_image.get_rect()

    rotated_image_rect.centerx = center_pos[0]
    rotated_image_rect.centery = center_pos[1]

    surface.blit(rotated_image, rotated_image_rect)
