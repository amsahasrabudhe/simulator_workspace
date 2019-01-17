cmake_minimum_required(VERSION 2.8.3)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#FIND_PACKAGE(CUDA REQUIRED)

set( packages
    
)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    ${packages}
)

#SET(CUDA_NVCC_FLAGS "-arch=sm_61" CACHE STRING "nvcc flags" FORCE)
#SET(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
#SET(LIB_TYPE STATIC)

###########
## Build ##
###########

set( headers
    types/EgoVehicle.hpp
    types/Pose2D.hpp
    types/Vehicle.hpp

    configs/PlannerConfig.hpp
)

set( libraries
    ${catkin_LIBRARIES}
)

add_library(parallel_mp
    ${headers}
)

## Specify libraries to link a library or executable target against
target_link_libraries(parallel_mp
    ${libraries}
)

set_target_properties(parallel_mp PROPERTIES LINKER_LANGUAGE CXX)