/// @file This file contains Host code for CUDA based parallel Motion planning algorithm

#ifndef NONHOLONOMIC_PARALLEL_ASTAR_HPP
#define NONHOLONOMIC_PARALLEL_ASTAR_HPP

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
        void localize(const std::int32_t& known_current_lane, const std::uint32_t& known_nearest_lane_point);

        /// @brief
        std::uint32_t findNearestLanePointByLaneId(const std::uint8_t& lane_id);

        /// @brief
        std::vector<Pose2D> getLanePointsForPolyFit(const std::uint32_t& lane_point_id);

    private:     /// variables

        std::shared_ptr<OverallInfo>    m_overall_info;

        PlannerConfig                   m_cfg;
};

}

#endif
