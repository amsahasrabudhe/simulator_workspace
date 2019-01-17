/// @file This file contains the top level class which can use multiple other classes related to Motion planning algorithms

#include <lib_motion_planning/types/EgoVehicle.hpp>
#include <lib_motion_planning/configs/PlannerConfig.hpp>

#include <simulator_msgs/TrafficVehicles.h>
#include <ros/ros.h>

namespace mp
{

class Planner
{
    public:
        /// @brief Constructor for top level planner class
        Planner(const ros::NodeHandle& nh);

        /// @brief Init function to start publishers, subsribers and timers
        void init();

        /// @brief Timer based update function for planner
        void update(const ros::TimerEvent& event);

    private:

        /// @brief
        void setupEgoVehicle();

        /// @brief
        void updateEgoVehicleState();

        /// @brief
        void publishEgoVehicleState();

        /// @brief Callback function for incoming traffic vehicles
        void trafficStatesReceived(const simulator_msgs::TrafficVehicles::ConstPtr& data);

    private:
        ros::NodeHandle m_nh;

        ros::Publisher  m_ego_state_pub;
        ros::Subscriber m_traffic_states_sub;

        ros::Timer      m_update_timer;

        PlannerConfig   m_config;

        EgoVehicle      m_ego_state;

        ros::Time       m_last_update_time;
};

}
