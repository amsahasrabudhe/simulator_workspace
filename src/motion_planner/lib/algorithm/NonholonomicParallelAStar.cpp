
#include "NonholonomicParallelAStar.hpp"

#include <eigen3/unsupported/Eigen/Splines>

namespace mp
{

NonholonomicParallelAStar::NonholonomicParallelAStar(const std::shared_ptr<OverallInfo>& overall_info, const PlannerConfig& cfg):
    m_overall_info(overall_info),
    m_cfg(cfg)
{

}

void NonholonomicParallelAStar::initialize()
{
    // Find current lane
    localize( m_overall_info->mp_info.current_lane, m_overall_info->nearest_lane_point_with_index.first );
}

void NonholonomicParallelAStar::update()
{

}

void NonholonomicParallelAStar::localize(const std::int32_t& known_current_lane, const std::uint32_t& known_nearest_lane_point)
{
    std::int8_t curr_lane = -1;
    std::pair<std::uint32_t, Pose2D> nearest_lane_point;

    double leastDist = 9999;
    for (std::int32_t i = known_current_lane; i < m_overall_info->road_info.num_lanes; ++i)
    {
        LaneInfo lane = m_overall_info->road_info.lanes[i];
        for (std::uint32_t j = known_nearest_lane_point; j < lane.lane_points.size(); ++j)
        {
            double dist = m_overall_info->ego_state->pose.distFrom( lane.lane_points[j] );
            if (dist < leastDist)
            {
                curr_lane = static_cast<std::int8_t>(i);
                nearest_lane_point = std::make_pair(j, lane.lane_points[j]);

                leastDist = dist;
            }
            else if(dist > leastDist)   // Check if we are moving away from the nearest point in current lane
            {
                // Move to next lane
                break;
            }
        }
    }

    /// Update MPInfo
    m_overall_info->mp_info.current_lane = curr_lane;
    m_overall_info->mp_info.desired_lane = curr_lane;

    /// Update localization information in OverallInfo
    m_overall_info->nearest_lane_point_with_index = nearest_lane_point;

    std::cout<<m_overall_info->nearest_lane_point_with_index.second.toString()<<std::endl;
}

std::uint32_t NonholonomicParallelAStar::findNearestLanePointByLaneId(const uint8_t &lane_id)
{

}

std::vector<Pose2D> NonholonomicParallelAStar::getLanePointsForPolyFit(const uint32_t &lane_point_id)
{

}

}
