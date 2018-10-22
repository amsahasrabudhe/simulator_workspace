#!/bin/bash

screen -X -S roscore quit
screen -X -S vehicle_description quit
screen -X -S motion_planner quit
screen -X -S simulator quit
#screen -X -S collision_detector quit

