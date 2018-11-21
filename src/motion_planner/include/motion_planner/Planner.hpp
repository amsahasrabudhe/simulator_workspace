/// @file This file contains the top level class which can use multiple other classes related to Motion planning algorithms

#include "EgoVehicle.hpp"
#include "PlannerConfig.hpp"

#include <ros/ros.h>

namespace mp
{

class Planner
{
    public:
        Planner(const ros::NodeHandle& nh);

        void init();

        void update(const ros::TimerEvent& event);

    private:

        void setupEgoVehicle();

        void publishEgoVehicleState();

    private:
        ros::NodeHandle m_nh;

        ros::Publisher  m_ego_state_pub;
        ros::Timer      m_update_timer;

        PlannerConfig   m_config;

        EgoVehicle      m_ego_state;
};

}
