cmake_minimum_required(VERSION 2.8.3)
project(libraries_builder)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

# Set the module path in which to search
SET( CMAKE_MODULE_PATH 
    ${CMAKE_MODULE_PATH} 
    ${CMAKE_SOURCE_DIR}/../libraries/CMakeModules
)

INCLUDE( MotionPlanning )

ADD_SUBDIRECTORY( ${CMAKE_SOURCE_DIR}/../libraries/core/lib_motion_planning lib_motion_planning )

catkin_package()

#############
## Install ##
#############

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/..
        USE_SOURCE_PERMISSIONS FILES_MATCHING PATTERN "lib_*.so*"
)
