/// @file This file contains configuration information related to motion planner

#include <string>

namespace mp
{

class PlannerConfig
{
    public:     /// functions
        PlannerConfig():
            ego_veh_state_out_topic("/ego_veh_state"),
            update_time_s(0.02)
        {

        }

    public:     /// variables

        std::string     ego_veh_state_out_topic;

        double          update_time_s;

};

}
