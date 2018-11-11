#!/usr/bin/python

import os
import rospy
import pygame
import yaml

from helper_classes.vehicle import Vehicle

def loadParam(param_name, default):
    return rospy.get_param(param_name, default)


def loadEnvironment(sim_obj):

    # Load road related information from the yaml file
    with open(sim_obj.sim_config.env_yaml_file, "r") as env_file:
        sim_obj.env_data = yaml.load(env_file)

    sim_obj.auto_car = setupAutonomousVehicle(sim_obj)

    # Upload scene information to ros parameter server for other packages to use
    rospy.set_param("sim_scene_data", sim_obj.env_data)

def loadImage(sim_obj, filename):

    image_path = os.path.join(sim_obj.current_dir, filename)
    return pygame.image.load(image_path).convert_alpha()


def checkPyGameQuit():

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            rospy.signal_shutdown("QUIT requested")


def setupAutonomousVehicle(sim_obj):

    auto_car_id      = 0
    auto_car_pos     = ( sim_obj.env_data['autonomous_vehicle_pose']['x'], sim_obj.env_data['autonomous_vehicle_pose']['y'] )
    auto_car_heading = sim_obj.env_data['autonomous_vehicle_pose']['heading']

    auto_car = Vehicle(veh_id=auto_car_id, veh_init_pos=auto_car_pos, veh_init_theta=auto_car_heading)
    auto_car.max_vel = loadParam("/vehicle_description/max_velocity_mps", 17.8816)
    auto_car.max_accel = loadParam("/vehicle_description/max_acceleration_mps2", 4.0)
    auto_car.max_steering_angle = loadParam("/vehicle_description/max_steering_angle_degree", 40.0)

    return auto_car


def rotateAndBlitImage(surface, image, center_pos, heading_angle):

    rotated_image = pygame.transform.rotozoom(image, heading_angle, 1)
    rotated_image_rect = rotated_image.get_rect()

    rotated_image_rect.centerx = center_pos[0]
    rotated_image_rect.centery = center_pos[1]

    surface.blit(rotated_image, rotated_image_rect)