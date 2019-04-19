#ifndef SERIAL_COST_CALCULATIONS_HPP
#define SERIAL_COST_CALCULATIONS_HPP

#include <stdio.h>

#include "astar/Node.hpp"
#include "lib/helper_functions/motion_planning_helper_functions.hpp"
#include "lib/types/OverallInfo.hpp"
#include "lib/types/Vec2D.hpp"

namespace serial_mp
{
    void calculateCost(std::vector<mp::Node>& child_nodes, const mp::PlannerConfig& config, const std::shared_ptr<mp::OverallInfo>& overall_info);

    std::vector<mp::Vec2D> calculateRoadPolygon(const mp::RoadInfo& road_info);

    std::vector<mp::Vec2D> calculateTrafficPolygons(const std::vector<mp::Vehicle>& traffic);
}

#endif //SERIAL_COST_CALCULATIONS_HPP
