#!/usr/bin/python

class SimulatorConfig:

    def __init__(self):

        self.window_width               = 1280
        self.window_height              = 720

        self.pixels_per_meter           = 14.16

        self.env_yaml_file              = "resources/environments/two_lane_straight_two_cars.yaml"
        self.env_bg_image_file          = "resources/environments/two_lane_straight.png"

        self.autonomous_car_image_file  = "resources/autonomous_car_small.png"
        self.traffic_car_image_file     = "resources/traffic_car_small.png"

        self.display_update_duration_s  = 0.02
