cmake_minimum_required(VERSION 2.8.3)
project(libraries_builder)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_planner
   CATKIN_DEPENDS roslaunch
#  DEPENDS system_lib
)

#############
## Install ##
#############

## Mark cpp header files for installation
 install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
