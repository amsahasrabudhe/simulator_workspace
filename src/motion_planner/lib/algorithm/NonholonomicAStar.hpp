/// @file This file contains Host code for CUDA based parallel Motion planning algorithm

#ifndef NONHOLONOMIC_PARALLEL_ASTAR_HPP
#define NONHOLONOMIC_PARALLEL_ASTAR_HPP

#include <eigen3/unsupported/Eigen/Splines>
#include <iostream>
#include <string>

#include "lib/configs/PlannerConfig.hpp"
#include "lib/types/OverallInfo.hpp"

namespace mp
{

class NonholonomicParallelAStar
{
    public:     /// functions

        /// @brief
        NonholonomicParallelAStar(const std::shared_ptr<OverallInfo>& overall_info, const PlannerConfig& cfg);

        /// @brief
        void initialize();

        /// @brief
        void update();

    private:

        /// @brief
        void localize(const std::size_t& known_current_lane, const std::size_t& known_nearest_lane_point_index);

        /// @brief
        Eigen::Spline3d getSpline( const std::vector<Pose2D>& points );

        /// @brief
        std::vector<Pose2D> getLanePointsForPolyFit();

    private:     /// variables

        std::shared_ptr<OverallInfo>    m_overall_info;

        PlannerConfig                   m_cfg;
};

}

#endif
