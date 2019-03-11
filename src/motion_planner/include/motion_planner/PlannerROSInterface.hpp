/// @file This file contains the top level class which can use multiple other classes related to Motion planning algorithms

#ifndef PLANNER_ROS_INTERFACE_HPP
#define PLANNER_ROS_INTERFACE_HPP

#include "PlannerVisualizer.hpp"
#include "lib/algorithm/NonholonomicAStar.hpp"

#include <lib/types/OverallInfo.hpp>
#include <lib/configs/PlannerConfig.hpp>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <simulator_msgs/EgoVehicle.h>
#include <simulator_msgs/TrafficVehicles.h>

namespace mp
{

class PlannerROSInterface
{
    public:
        /// @brief Constructor for top level planner class
        PlannerROSInterface(const ros::NodeHandle& nh);

        /// @brief Init function to start publishers, subsribers and timers
        void initialize();

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

        /// @brief Broadcast required frame transformations
        void broadcastTransforms();

        /// @brief Callback function for incoming traffic vehicles
        void trafficStatesReceived(const simulator_msgs::TrafficVehicles::ConstPtr& data);

    private:
        ros::NodeHandle                             m_nh;

        ros::Publisher                              m_ego_state_pub;
        ros::Subscriber                             m_traffic_states_sub;

        tf2_ros::TransformBroadcaster               m_tf_broadcaster;

        ros::Timer                                  m_update_timer;

        PlannerConfig                               m_config;
        std::shared_ptr<OverallInfo>                m_overall_info;
        std::shared_ptr<NonholonomicParallelAStar>  m_parallel_mp_algo;
        std::shared_ptr<PlannerVisualizer>          m_visualizer;

        ros::Time                                   m_last_update_time;
};

}

#endif
