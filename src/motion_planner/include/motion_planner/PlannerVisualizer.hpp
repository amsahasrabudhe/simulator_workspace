/// @file This file contains class for visualizing motion planning algorithm

#ifndef PLANNER_VISUALIZER_HPP
#define PLANNER_VISUALIZER_HPP

#include <lib/types/OverallInfo.hpp>
#include <lib/configs/PlannerConfig.hpp>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace mp
{

class PlannerVisualizer
{
    public:
        /// @brief Constructor for visualizer class
        PlannerVisualizer(const ros::NodeHandle& nh, const std::shared_ptr<OverallInfo>& info, const PlannerConfig& cfg);

        /// @brief Init function for publishers and timers
        void initialize();

        /// @brief Timer based update function for visualization
        void update();

    private:
        /// @brief Function to publish visualization messages
        void addVisualization();

    private:
        ros::NodeHandle                                     m_nh;

        ros::Publisher                                      m_visualization_pub;

        PlannerConfig                                       m_cfg;
        std::shared_ptr<OverallInfo>                        m_overall_info;

        boost::shared_ptr<visualization_msgs::MarkerArray>  m_vis_msg;
};

}

#endif
