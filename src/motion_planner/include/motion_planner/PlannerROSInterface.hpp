/// @file This file contains the top level class which can use multiple other classes related to Motion planning algorithms

#ifndef PLANNER_ROS_INTERFACE_HPP
#define PLANNER_ROS_INTERFACE_HPP

#include <lib/types/OverallInfo.hpp>
#include <lib/configs/PlannerConfig.hpp>

#include <simulator_msgs/TrafficVehicles.h>
#include <ros/ros.h>

namespace mp
{

class PlannerROSInterface
{
    public:
        /// @brief Constructor for top level planner class
        PlannerROSInterface(const ros::NodeHandle& nh);

        /// @brief Init function to start publishers, subsribers and timers
        void init();

        /// @brief Timer based update function for planner
        void update(const ros::TimerEvent& event);

    private:

        /// @brief Load config for planner from ROS parameter server
        void loadConfigFromParameterServer();        

        /// @brief Load scene details from ROS parameter server uploaded by simulator node
        void loadSceneDetailsFromParameterServer();

        /// @brief Setup ego vehicle using data from ros parameters from simulator
        void setupEgoVehicle();

        /// @brief Function to update ego vehicle state after current timestep
        void updateEgoVehicleState();

        /// @brief Function to publish updated ego vehicle state
        ///        Simulator subscribes to the new state and displays the updated position
        void publishEgoVehicleState();

        /// @brief Callback function for incoming traffic vehicles
        void trafficStatesReceived(const simulator_msgs::TrafficVehicles::ConstPtr& data);

    private:
        ros::NodeHandle                 m_nh;

        ros::Publisher                  m_ego_state_pub;
        ros::Subscriber                 m_traffic_states_sub;

        ros::Timer                      m_update_timer;

        PlannerConfig                   m_config;

        std::shared_ptr<OverallInfo>    m_overall_info;

        ros::Time                       m_last_update_time;
};

}

#endif