﻿/// @file This file contains configuration information related to motion planner

#ifndef PLANNER_CONFIG_HPP
#define PLANNER_CONFIG_HPP

#include <string>

namespace mp
{

struct PlannerConfig
{
    public:

        /// Vehicle dimension
        double wheel_base{2.81};
        double height{1.451};

        /// Topic configs
        std::string     ego_veh_state_out_topic{"/ego_veh_state"};          ///< Topic name to publish ego vehicle state
        std::string     traffic_veh_states_in_topic{"/traffic_veh_states"}; ///< Topic name to receive traffic vehicle information
        std::string     visualization_topic{"/planner/visualization"};      ///< Topic name to publish visualization messages
        std::string     planned_path_topic{"/planner/planned_path"};        ///< Topic name to publish planned path

        /// Max value configs
        double          max_vel_mps{17.8816};           ///< Maximum velocity
        double          max_accel_mpss{3.0};            ///< Maximum acceleration
        double          max_jerk_mpsss{4.0};            ///< Maximum jerk
        double          max_steering_rad{0.698132};     ///< Maximum steering angle in radians

        double          braking_accel{-1.0};            ///< Braking accel value

        std::size_t     threads_per_block{128};
        double          dist_to_goal{12.0};

        double          child_node_dt{1.0};             ///< dt used for child node creation
        double          update_time_s{0.05};            ///< Planner update time
        double          plan_path_time_s{0.75};         ///< Plan path call time
};

}

#endif
