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
            update_time_s(0.02)
        {

        }

    public:     /// variables

        std::string     ego_veh_state_out_topic;
        std::string     traffic_veh_states_in_topic;

        double          max_vel;
        double          max_accel;
        double          max_steering;
        double          max_jerk;

        double          update_time_s;
};

}

#endif