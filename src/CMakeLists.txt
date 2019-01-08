cmake_minimum_required(VERSION 2.8.3)

SET(CMAKE_BUILD_TYPE RELEASE)

# Set libraries folder
set(LIBRARIES_DIR "${CMAKE_CURRENT_LIST_DIR}/../libraries")

include($ENV{ROS_ROOT}/../catkin/cmake/toplevel.cmake REQUIRED)