#!/usr/bin/python3

"""!@file Contains code to generate json file containing road + traffic related information for simulation.
"""

import pygame
import os
import math
import yaml

# Define dimensions of the pygame screen
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720

# Define pixels per meter for the environment being generated
PIXELS_PER_METER = 4.26

VEHICLE_LENGTH_M = 4.97
VEHICLE_WIDTH_M = 2.1

# Load path of current directory to be used for loading other resources
current_dir = os.path.dirname(os.path.abspath(__file__))

scaled_image_size = (int(VEHICLE_LENGTH_M*PIXELS_PER_METER), int(VEHICLE_WIDTH_M*PIXELS_PER_METER))

# Load image of autonomous car
autonomous_car_image_path = os.path.join(current_dir, "resources/autonomous_car_small.png")
autonomous_car_image      = pygame.image.load(autonomous_car_image_path)
autonomous_car_image = pygame.transform.scale(autonomous_car_image, scaled_image_size)

# Load image of traffic car
traffic_car_image_path = os.path.join(current_dir, "resources/traffic_car_small.png")
traffic_car_image      = pygame.image.load(traffic_car_image_path)
traffic_car_image      = pygame.transform.scale(traffic_car_image, scaled_image_size)

# Basic instructions for the user
print("\nAfter you enter the name of environment file, pygame window will appear with the given environment as background")
print("1) First the position of autonomous car needs to be selected")
print("2) Now as you move the mouse, the car will rotate, click again to fix the initial orientation of the autonomous car (previously added car might disappear, but its still captured internally)")
print("3) After that input the number of traffic cars and repeat the same procedure for the traffic cars")
print("4) Close the window when done\n")

# Get name for the file that will be generated at the end
ROAD_DATA_FILE_NAME = input("Enter the name for environment/road data file (without file extension): ")

# Load image of road to be used as background
road_image_path = os.path.join(current_dir, "generated_environments/"+ROAD_DATA_FILE_NAME+".png")
road_image      = pygame.image.load(road_image_path)

# Load file path which contains road related information
environment_yaml_file_path = os.path.join(current_dir, "generated_environments/"+ROAD_DATA_FILE_NAME+".yaml")

# Load road related information from the yaml file
with open(environment_yaml_file_path, "r") as env_file:
    env_data = yaml.safe_load(env_file)

# Initialize the pygame window once number of lanes are known
pygame.init()
pygame.display.set_caption("2D Car Simulator - Traffic Scene generator")
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
screen.blit(road_image, (0,0))

def invertY(y_val):
    return (WINDOW_HEIGHT - y_val)

def getEgoPose(vehicle_image):

    # Run the loop infinitely to get all event inputs and exit on window close or completion of road generation procedure
    while True:

        # Capture all pygame events
        for event in pygame.event.get():

            # Get mouse click to get center point location of car
            if event.type == pygame.MOUSEBUTTONDOWN:

                car_center = event.pos

                car_image = pygame.transform.rotozoom(vehicle_image, 0, 1)
                car_rect = car_image.get_rect()

                car_rect.centerx = car_center[0]
                car_rect.centery = car_center[1]

                screen.blit(car_image, car_rect)

                # Update the display to visualize current car
                pygame.display.update()

                while True:

                    # Capture all pygame events
                    for event in pygame.event.get():

                        if event.type == pygame.MOUSEMOTION:

                            # Clear image with bacground image
                            screen.blit(road_image, (0, 0))

                            angle_radians = math.atan2(invertY(event.pos[1]) - invertY(car_center[1]),
                                                   event.pos[0] - car_center[0])

                            angle = math.degrees(angle_radians)

                            car_image_rotated = pygame.transform.rotate(vehicle_image, angle)
                            car_rotated_rect = car_image_rotated.get_rect()

                            car_rotated_rect.centerx = car_center[0]
                            car_rotated_rect.centery = car_center[1]

                            screen.blit(car_image_rotated, car_rotated_rect)

                            pygame.display.update()

                        elif event.type == pygame.MOUSEBUTTONDOWN:

                            angle_radians = math.atan2(invertY(event.pos[1]) - invertY(car_center[1]),
                                                       event.pos[0] - car_center[0])

                            angle = math.degrees(angle_radians)

                            # invert y before writing to the file
                            car_center_inverted_y = (car_center[0], invertY(car_center[1]))

                            return car_center_inverted_y, angle

                        # Check if window is closed
                        elif event.type == pygame.QUIT:
                            quit()

            # Check if window is closed
            elif event.type == pygame.QUIT:
                quit()

        pygame.display.update()


if __name__ == "__main__":

    print("Click anywhere inside the roads to place the autonomous vehicle\n")

    # Get start pose for the autonomous vehicle
    position, heading = getEgoPose(autonomous_car_image)

    # Add information to the yaml data
    env_data['ego_veh_pose'] = {'x': position[0]/PIXELS_PER_METER, 'y': position[1]/PIXELS_PER_METER, 'heading': math.radians(heading)}

    NUM_TRAFFIC_VEHICLES = int(input("Enter the number of traffic vehicles: "))

    env_data['traffic_vehicles'] = []

    for vehicle_id in range(NUM_TRAFFIC_VEHICLES):

        # Get start pose for the autonomous vehicle
        position, heading = getEgoPose(traffic_car_image)

        # Add information to the yaml data
        traffic_vehicle_data = {}
        traffic_vehicle_data['vehicle_id']       = 100 + vehicle_id
        traffic_vehicle_data['vehicle_pose']     = {'x': position[0]/PIXELS_PER_METER, 'y': position[1]/PIXELS_PER_METER, 'heading': math.radians(heading)}
        traffic_vehicle_data['initial_velocity'] = float(input("Enter vehicle's initial velocity along lane in m/s: "))

        env_data['traffic_vehicles'].append( traffic_vehicle_data )

    OUTPUT_FILE_NAME = input("Enter file name for output environment file (image will not be saved): ")

    with open("generated_environments/"+OUTPUT_FILE_NAME+".yaml", 'w') as outfile:
        yaml.safe_dump(env_data, outfile)