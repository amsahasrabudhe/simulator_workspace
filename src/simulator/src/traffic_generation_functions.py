#!/usr/bin/python3

from simulator_helper_functions import *


def updateTrafficVehiclesPositions(sim_obj, now):

    dt = now - sim_obj.last_update_time

    for vehicle in sim_obj.traffic:

        curr_theta = vehicle.pose.heading
        curr_vel = vehicle.vel

        vehicle.pose.x += dt * curr_vel * math.cos(curr_theta)
        vehicle.pose.y += dt * curr_vel * math.sin(curr_theta)

        vehicle.pose.heading = curr_theta + dt * (curr_vel/vehicle.wheel_base)*math.tan(vehicle.steering)

        vehicle.vel += dt * vehicle.accel

    sim_obj.last_update_time = now