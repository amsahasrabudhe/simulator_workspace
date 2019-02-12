
#include "NonholonomicParallelAStar.hpp"

namespace mp
{

NonholonomicParallelAStar::NonholonomicParallelAStar(const std::shared_ptr<OverallInfo>& overall_info, const PlannerConfig& cfg):
    m_overall_info(overall_info),
    m_cfg(cfg)
{

}

void NonholonomicParallelAStar::initialize()
{

}

void NonholonomicParallelAStar::update()
{

}

std::uint32_t NonholonomicParallelAStar::findNearestLanePointByLaneId(const uint8_t &lane_id)
{

}

std::vector<Pose2D> NonholonomicParallelAStar::getLanePointsForPolyFit(const uint32_t &lane_point_id)
{

}

}
