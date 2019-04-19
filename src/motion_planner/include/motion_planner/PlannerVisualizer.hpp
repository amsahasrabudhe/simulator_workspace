/// @file This file contains class for visualizing motion planning algorithm

#ifndef PLANNER_VISUALIZER_HPP
#define PLANNER_VISUALIZER_HPP

#include <lib/types/OverallInfo.hpp>
#include <lib/configs/PlannerConfig.hpp>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

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

        void addSplineLanePointsVis();
        void addLanesVis();
        void addEgoVehicleVis();
        void addTrafficVis();
        void addRoadPolygonVis();
        void addEgoPolygonVis();

        void addPlannedPathVis();
        void addCurrChildNodesVis();

        void addPlannedPathSplineVis();

    private:
        ros::NodeHandle                                     m_nh;

        ros::Publisher                                      m_visualization_pub;
        ros::Publisher                                      m_path_pub;

        PlannerConfig                                       m_cfg;
        std::shared_ptr<OverallInfo>                        m_overall_info;

        boost::shared_ptr<visualization_msgs::MarkerArray>  m_vis_msg;
        boost::shared_ptr<nav_msgs::Path>                   m_planned_path_msg;
};

}

#endif
