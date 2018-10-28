"""!@file Contains code to generate json file containing road related information for simulation.
"""

import pygame
import os
import math
import numpy as np

LANE_WIDTH = 60

BLOCK_WIDTH = 8
HALF_BLOCK_DIAG = 60.53/2

current_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(current_dir, "resources/block.png")

block_image = pygame.image.load(image_path)

print "After you input the number of lanes, an empty window will open."
print "1) Keep left mouse button pressed move cursor slowly inside the window (left to right) where you want the lane to be marked."
print "2) Close the window once you are done\n"

#NUM_LANES = input("Enter the number of lanes (2-4) :")

pygame.init()
screen = pygame.display.set_mode((1280, 720))
screen.fill((255,255,255))

curr_angle, prev_angle = 0.0, 0.0
prev_pos = (0, 0)

def interpolate(input_val, input_min, input_max, output_min, output_max):

    output_val = input_val * (output_max-output_min)/(input_max-input_min) + output_min
    return output_val


initialized = False

x_list = []
y_list = []

while True:

    for event in pygame.event.get():

        if event.type == pygame.MOUSEBUTTONDOWN:

            x_list.append(event.pos[0])
            y_list.append(event.pos[1])

            screen.fill((0, 255, 0), ((event.pos[0], event.pos[1]), (5, 5)))

        elif event.type == pygame.KEYDOWN and event.key == pygame.K_g:

            x_list = [0] + x_list
            y_list = [y_list[0]] + y_list

            prev_pos = (x_list[0]+1, y_list[0])

            poly_coordinates = np.polyfit(x_list, y_list, 9)
            poly_eval = np.poly1d(poly_coordinates)

            for x in range(x_list[0], x_list[-1]):

                y = poly_eval(x)
                angle = math.degrees( math.atan2( y-prev_pos[1], x-prev_pos[0] ) )

                block_image_rotated = pygame.transform.rotozoom(block_image, -angle, 1)
                rotated_rect = block_image_rotated.get_rect()

                rotated_rect.centerx = x
                rotated_rect.centery = y

                screen.blit(block_image_rotated, rotated_rect)

                prev_pos = (x,y)

            for x in range(x_list[0], x_list[-1]):

                y = poly_eval(x)

                screen.fill((255, 255, 255), ((x,y), (2,2)))

                prev_pos = (x, y)

        elif event.type == pygame.QUIT:
            quit()

    pygame.display.update()
