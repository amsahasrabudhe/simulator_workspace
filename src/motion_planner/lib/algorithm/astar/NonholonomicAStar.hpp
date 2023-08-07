/// @file This file contains Host code for CUDA based parallel Motion planning algorithm

#ifndef NONHOLONOMIC_ASTAR_HPP
#define NONHOLONOMIC_ASTAR_HPP

#include <eigen3/unsupported/Eigen/Splines>
#include <iostream>
#include <queue>
#include <ros/ros.h>
#include <ros/time.h>
#include <string>

#include "lib/configs/PlannerConfig.hpp"
#include "lib/types/OverallInfo.hpp"

namespace mp
{

class NonholonomicAStar
{
    public:     /// functions

        /// @brief
        NonholonomicAStar(const ros::NodeHandle& nh, const std::shared_ptr<OverallInfo>& overall_info, const PlannerConfig& cfg);

        /// @brief
        void initialize();

        /// @brief
        void planPath();

    private:

        /// @brief
        void addToOpenList(const Node& node);

        /// @brief
        void addToClosedList(const Node& node);

        /// @brief
        void localizeOnRoad();

    private:     /// variables

        ros::NodeHandle                 m_nh;

        std::shared_ptr<OverallInfo>    m_overall_info;
        PlannerConfig                   m_cfg;

        bool                            m_planner_failed;

        std::vector<Node>               m_open_list;
        std::vector<Node>               m_closed_list;
};

}

#endif
