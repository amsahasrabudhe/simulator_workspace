#!/usr/bin/python3

class SimulatorConfig:

    def __init__(self):

        self.ego_veh_state_topic        = "/ego_veh_state"
        self.traffic_states_topic       = "/traffic_veh_states"

        self.window_width               = 1280
        self.window_height              = 720

        self.veh_wheel_base_m           = 2.81
        self.veh_length_m               = 4.97
        self.veh_width_m                = 2.1

        self.pixels_per_meter           = 4.26

        self.env_yaml_file              = "resources/environments/two_lane_straight_two_cars.yaml"
        self.env_bg_image_file          = "resources/environments/two_lane_straight.png"

        self.ego_veh_image_file         = "resources/ego_veh_small.png"
        self.traffic_veh_image_file     = "resources/traffic_veh_small.png"

        self.render                     = False

        self.display_update_duration_s  = 0.02
