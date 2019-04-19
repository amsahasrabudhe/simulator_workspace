cmake_minimum_required(VERSION 2.8.3)
project(motion_planner)

set(CXX g++-4.9)
set(C gcc-4.9)

find_package(CUDA REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

set( packages
    message_runtime
    roscpp
    roslaunch
    simulator_msgs
    tf2
    tf2_ros
)

find_package(catkin REQUIRED COMPONENTS
    ${packages}
)

# Set the module path in which to search
set( CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH} 
    ${CMAKE_SOURCE_DIR}/motion_planner
)

include( MotionPlanning )

set(CMAKE_CUDA_COMPILER /usr/local/cuda-9.0/bin/nvcc)
set(CUDA_NVCC_FLAGS "-arch=sm_61" CACHE STRING "nvcc flags" FORCE)
set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
set(LIB_TYPE SHARED)

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
  ${CUDA_INCLUDE_DIRS}
)

##################################################

set( headers
    include/motion_planner/PlannerROSInterface.hpp
    include/motion_planner/PlannerVisualizer.hpp
)

set( source
    src/main.cpp
    src/PlannerROSInterface.cpp
    src/PlannerVisualizer.cpp
)

add_executable(motion_planner
    ${headers}
    ${source}
)

##################################################

set( cuda_lib_headers
    lib/algorithm/parallel_cost_calculations.cuh
)

set( cuda_lib_source
    lib/algorithm/parallel_cost_calculations.cu
)

cuda_add_library( cuda_mp ${LIB_TYPE}
    ${cuda_lib_headers}
    ${cuda_lib_source}
)

##################################################

set( lib_headers
    lib/algorithm/astar/NonholonomicAStar.hpp
    lib/algorithm/astar/Node.hpp
    lib/algorithm/serial_cost_calculations.hpp

    lib/configs/PlannerConfig.hpp

    lib/helper_functions/boost_geometry_helper_functions.hpp
    lib/helper_functions/motion_planning_helper_functions.hpp
    lib/helper_functions/safety_helper_functions.hpp

    lib/types/EgoVehicle.hpp
    lib/types/LaneInfo.hpp
    lib/types/MPInfo.hpp
    lib/types/OverallInfo.hpp
    lib/types/Pose2D.hpp
    lib/types/RoadInfo.hpp
    lib/types/Vehicle.hpp
)

set( lib_source
    lib/algorithm/astar/NonholonomicAStar.cpp
    lib/algorithm/serial_cost_calculations.cpp
)

add_library( mp
    ${lib_headers}
    ${lib_source}
)

target_link_libraries(mp
    cuda_mp
)

##################################################

set( libraries
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${CUDA_LIBRARIES}
    mp
)

add_dependencies(motion_planner mp)

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

install( TARGETS mp
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install( TARGETS cuda_mp
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   PATTERN ".svn" EXCLUDE
 )
