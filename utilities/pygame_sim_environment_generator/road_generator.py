#!/usr/bin/python3

"""!@file Contains code to generate json file containing road related information for simulation.

    NOTE: Y coordinates are inverted to make it similar to world 2D coordinate system with 0,0 starting from
          bottom-left corner
"""

import pygame
import os
import math
import numpy as np
import yaml

# Define dimensions of the pygame screen
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720

# Define pixels per meter for the environment being generated
PIXELS_PER_METER = 4.26

# Define lane width (here height of image since road is represented horizontally)
LANE_WIDTH_M = 3.6
LANE_WIDTH_PIXELS = int(LANE_WIDTH_M*PIXELS_PER_METER)

# Define the distance between two lane points (number of pixels) written to the json file
MAX_LANE_POINT_SEP_M = 15.0
LANE_POINT_DIST_PIXELS = int(MAX_LANE_POINT_SEP_M*PIXELS_PER_METER)

# Load image of block of road to create base image
current_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(current_dir, "resources/block.png")

block_image = pygame.image.load(image_path)

new_size = (int(LANE_WIDTH_PIXELS*block_image.get_width()/block_image.get_height()), LANE_WIDTH_PIXELS)
block_image = pygame.transform.scale(block_image, new_size)

# Basic instructions for the user
print("\nAfter you input the number of lanes, an empty window will open.")
print("1) Click points on the screen you want the road to go through (make sure they make a simple shape)")
print("2) Press 'g' to generate the road")
print("3) Close the screen once a satisfactory road is generated\n")

# Get number of lanes as input from user
NUM_LANES = int(input("Enter the number of lanes (2-4): "))

# Get name for the file that will be generated at the end
FILE_NAME = input("Enter the name for environment file (without file extension): ")

# Initialize the pygame window once number of lanes are known
pygame.init()
pygame.display.set_caption("2D Car Simulator - Road Generator")
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
screen.fill((255, 255, 255))

prev_pos = (0, 0)

x_list = []
y_list = []

def invertY(y_val):
    return (WINDOW_HEIGHT - y_val)

def writeSimulationEnvironmentYamlFile(x_max, poly_eval):

    global x_list, y_list, prev_pos

    road_data = {}
    road_data['lane_width'] = LANE_WIDTH_M
    road_data['lanes'] = []

    for lane_id in range(NUM_LANES):

        prev_pos = (x_list[0], y_list[0])

        road_data['lanes'].append({'lane_id' : lane_id, 'lane_points' : []})

        for x in range(LANE_POINT_DIST_PIXELS, x_max, LANE_POINT_DIST_PIXELS):

            y = int( poly_eval(x) )

            angle_radians = math.atan2(y - prev_pos[1], x - prev_pos[0])

            lane_point_info = {'x' : x/PIXELS_PER_METER, 'y' : (y - lane_id*LANE_WIDTH_PIXELS)/PIXELS_PER_METER, 'heading' : angle_radians}
            road_data['lanes'][lane_id]['lane_points'].append(lane_point_info)

            prev_pos = (x, y)

    with open("generated_environments/"+FILE_NAME+".yaml", 'w') as outfile:
        yaml.safe_dump(road_data, outfile)

    # Save current image with road and markings to be used as background for simulation
    pygame.image.save(screen, "generated_environments/"+FILE_NAME+".png")


def generateLanes():

    global x_list, y_list, prev_pos

    # Run the loop infinitely to get all event inputs and exit on window close or completion of road generation procedure
    while True:

        # Capture all pygame events
        for event in pygame.event.get():

            # Get all mouse press events to mark the way-points that create the road
            # Road is generated from these way-points using polynomial fit function of numpy library
            if event.type == pygame.MOUSEBUTTONDOWN:

                x_list.append(event.pos[0])
                y_list.append( invertY(event.pos[1]) )

                # Mark the point with green dot
                screen.fill((0, 255, 0), ((event.pos[0], event.pos[1]), (5, 5)))

            # Get key press event for 'g' which generates and draws the road based on the waypoints collected
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_g:

                # Check if atleast one point was selected for road generation procedure
                if len(x_list) == 0:

                    print("\nYou have not selected any points for the road.")
                    print("Exiting now...")
                    quit(1)

                # Add way-point with x=0 for the 1st way-point selected so that the road starts from the start of screen
                x_list = [0] + x_list
                y_list = [y_list[0]] + y_list

                prev_pos = (x_list[0], y_list[0])

                # Run polynomial fit algorithm on the selected points
                poly_coordinates = np.polyfit(x_list, y_list, 7)
                poly_eval = np.poly1d(poly_coordinates)

                x_max = max(x_list)

                # Generate lane point data for all lanes
                # This loop also draws the base road with given number of lanes
                for lane in range(NUM_LANES):

                    for x in range(x_max):

                        y = poly_eval(x)
                        angle = math.degrees( math.atan2( y-prev_pos[1], x-prev_pos[0] ) )

                        block_image_rotated = pygame.transform.rotozoom(block_image, angle, 1)
                        rotated_rect = block_image_rotated.get_rect()

                        y_draw = invertY(y)

                        rotated_rect.centerx = x
                        rotated_rect.centery = y_draw + lane*LANE_WIDTH_PIXELS

                        screen.blit(block_image_rotated, rotated_rect)

                        prev_pos = (x, y)

                # Update the display to visualize drawn roads
                pygame.display.update()

                # Draw lane separators and lane center markings
                for lane in range(NUM_LANES):

                    draw_lane_separator = True
                    for x in range(x_max):

                        y = poly_eval(x)
                        y_draw = invertY(y)

                        # Draw lane separator markings
                        if draw_lane_separator and lane != NUM_LANES-1:
                            lane_sep_pos = (x, int(y_draw + lane*LANE_WIDTH_PIXELS + LANE_WIDTH_PIXELS/2))
                            screen.fill((255, 255, 255), (lane_sep_pos, (1,1)))

                        # Draw lane center markings
                        if divmod(x, 45)[1] == 0:
                            screen.fill((255, 255, 0), ((x, y_draw + lane*LANE_WIDTH_PIXELS), (2, 2)))

                        # Toggle lane separator markings on/off every 30 pixels
                        if divmod(x, 30)[1] == 0:
                            draw_lane_separator = not draw_lane_separator

                writeSimulationEnvironmentYamlFile(x_max, poly_eval)

            # Check if window is closed
            elif event.type == pygame.QUIT:
                quit()

        pygame.display.update()


if __name__ == "__main__":
    generateLanes()