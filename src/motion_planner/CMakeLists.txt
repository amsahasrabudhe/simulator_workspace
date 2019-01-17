cmake_minimum_required(VERSION 2.8.3)
project(motion_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#FIND_PACKAGE(CUDA REQUIRED)

set( packages
    message_runtime
    roscpp
    roslaunch
    simulator_msgs
)

find_package(catkin REQUIRED COMPONENTS
    ${packages}
)

# Set the module path in which to search
SET( CMAKE_MODULE_PATH 
    ${CMAKE_MODULE_PATH} 
    ${CMAKE_SOURCE_DIR}/../libraries/CMakeModules
)

INCLUDE( MotionPlanning )

#SET(CUDA_NVCC_FLAGS "-arch=sm_61" CACHE STRING "nvcc flags" FORCE)
#SET(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
#SET(LIB_TYPE STATIC)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${packages}
)

###########
## Build ##
###########

# Add non build files for project viewing
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set( headers
    include/motion_planner/Planner.hpp
)

set( source
    src/main.cpp
    src/Planner.cpp
)

set( libraries
    ${catkin_LIBRARIES}
    parallel_mp
)

add_executable(motion_planner
    ${headers}
    ${source}
)

## Specify libraries to link a library or executable target against
target_link_libraries(motion_planner
    ${libraries}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install( TARGETS motion_planner
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   PATTERN ".svn" EXCLUDE
 )
