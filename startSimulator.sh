#!/bin/bash

echo "Starting car simulation"

stop_screens() {
	#screen -X -S roscore quit
	screen -X -S vehicle_description quit
	screen -X -S motion_planner quit
	screen -X -S simulator quit
}

echo "Killing all existing screens"
stop_screens

#echo "Starting roscore"
#screen -d -m -S roscore bash -c "source devel/setup.bash ; roscore; exec bash"

echo "Starting vehicle_description"
screen -d -m -S vehicle_description bash -c "source devel/setup.bash ; roslaunch vehicle_description vehicle_description.launch ; exec bash"

echo "Starting simulator node"
screen -d -m -S simulator bash -c "source devel/setup.bash ; roslaunch simulator simulator.launch ; exec bash"

sleep 2.5

echo "Starting motion_planner node"
screen -d -m -S motion_planner bash -c "source devel/setup.bash ; roslaunch motion_planner motion_planner.launch ; exec bash"
