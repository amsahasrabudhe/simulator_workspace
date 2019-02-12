cmake_minimum_required(VERSION 2.8.3)
project(motion_planner)

add_compile_options(-std=c++11)

#find_package(CUDA REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

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
    ${CMAKE_SOURCE_DIR}/motion_planner
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
  LIBRARIES parallel_mp
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

##################################################

set( headers
    include/motion_planner/PlannerROSInterface.hpp
)

set( source
    src/main.cpp
    src/PlannerROSInterface.cpp
)

add_executable(motion_planner
    ${headers}
    ${source}
)

##################################################

set( lib_headers
    lib/algorithm/NonholonomicParallelAStar.hpp

    lib/configs/PlannerConfig.hpp

    lib/types/EgoVehicle.hpp
    lib/types/LaneInfo.hpp
    lib/types/MPInfo.hpp
    lib/types/OverallInfo.hpp
    lib/types/Pose2D.hpp
    lib/types/RoadInfo.hpp
    lib/types/Vehicle.hpp
)

set( lib_source
    lib/algorithm/NonholonomicParallelAStar.cpp
)

add_library( parallel_mp
    ${lib_headers}
    ${lib_source}
)

##################################################

set( libraries
    ${catkin_LIBRARIES}
    parallel_mp
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
