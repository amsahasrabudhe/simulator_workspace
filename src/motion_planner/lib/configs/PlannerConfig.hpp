/// @file This file contains configuration information related to motion planner

#ifndef PLANNER_CONFIG_HPP
#define PLANNER_CONFIG_HPP

#include <string>

namespace mp
{

class PlannerConfig
{
    public:     /// functions
        PlannerConfig():
            ego_veh_state_out_topic("/ego_veh_state"),
            traffic_veh_states_in_topic("/traffic_veh_states"),
            visualization_topic("/planner/visualization"),
            max_vel_mps(17.8816),
            max_accel_mpss(4.0),
            max_jerk_mpsss(1.0),
            max_steering_rad(0.698132),
            poly_fit_degree(2),
            poly_fit_min_lane_points(11),
            poly_fit_lane_points_behind_veh(1),
            poly_fit_speed_scale_factor(2.0),
            update_time_s(0.02)
        {

        }

    public:

        /// Topic configs
        std::string     ego_veh_state_out_topic;        ///< Topic name to publish ego vehicle state
        std::string     traffic_veh_states_in_topic;    ///< Topic name to receive traffic vehicle information
        std::string     visualization_topic;            ///< Topic name to publish visualization messages

        /// Max value configs
        double          max_vel_mps;                    ///< Maximum velocity
        double          max_accel_mpss;                 ///< Maximum acceleration
        double          max_jerk_mpsss;                 ///< Maximum jerk
        double          max_steering_rad;               ///< Maximum steering angle in radians

        /// Curve fitting configs
        std::uint8_t    poly_fit_degree;                    ///< Degree
        std::size_t     poly_fit_min_lane_points;           ///< Min number of lane points used for curve fitting
        std::size_t     poly_fit_lane_points_behind_veh;    ///< Number of lane points from behind the vehicle used for smooth transition in curve fitting
        double          poly_fit_speed_scale_factor;        ///< Scaling factor for calculating distance for total poly fit

        double          update_time_s;                  ///< Planner algorithm update time
};

}

#endif
