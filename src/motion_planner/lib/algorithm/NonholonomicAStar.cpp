
#include "NonholonomicAStar.hpp"
#include "parallel_cost_calculations.cuh"

namespace mp
{

NonholonomicAStar::NonholonomicAStar(const std::shared_ptr<OverallInfo>& overall_info, const PlannerConfig& cfg):
    m_overall_info(overall_info),
    m_cfg(cfg)
{

}

void NonholonomicAStar::initialize()
{
    mp::EgoVehicle ego = *m_overall_info->ego_state;
    Node *node = new Node(ego.pose.x, ego.pose.y, ego.pose.theta, ego.steering, ego.vel, ego.accel);

    cuda_mp::calculateCost(node, m_cfg);

    // Find current lane
    localize( m_overall_info->mp_info.current_lane, m_overall_info->nearest_lane_point_with_index.first );
}

void NonholonomicAStar::update()
{
    // Get Lane points based on localization information
    m_overall_info->curr_poly_lanepoints = getLanePointsForPolyFit();

    // Fit spline for given lane points
    m_overall_info->lane_center_spline = getSpline( m_overall_info->curr_poly_lanepoints );

}

void NonholonomicAStar::localize(const std::size_t& known_current_lane, const std::size_t& known_nearest_lane_point_index)
{
    std::size_t curr_lane = 0;
    std::pair<std::uint32_t, Pose2D> nearest_lane_point;

    // TODO: Optimize localize function to find nearest point

    double leastDist = 9999;
    for (std::size_t i = 0; i < m_overall_info->road_info.num_lanes; ++i)
    {
        LaneInfo lane = m_overall_info->road_info.lanes[i];
        for (std::size_t j = known_nearest_lane_point_index; j < lane.lane_points.size(); ++j)
        {
            double dist = m_overall_info->ego_state->pose.distFrom( lane.lane_points[j] );
            if (dist < leastDist)
            {
                curr_lane = i;
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
    //m_overall_info->mp_info.desired_lane = curr_lane;

    /// Update localization information in OverallInfo
    m_overall_info->nearest_lane_point_with_index = nearest_lane_point;
}

std::vector<Pose2D> NonholonomicAStar::getLanePointsForPolyFit()
{
    /// Get nearest lane point and lane id
    localize(m_overall_info->mp_info.current_lane, m_overall_info->nearest_lane_point_with_index.first);

    std::vector<Pose2D> points;

    std::size_t count = 0;
    std::size_t first_pt = m_overall_info->nearest_lane_point_with_index.first - m_cfg.poly_fit_lane_points_behind_veh;

    LaneInfo curr_lane = m_overall_info->road_info.lanes[m_overall_info->mp_info.current_lane];
    while (count < m_cfg.poly_fit_min_lane_points && (first_pt + count) <= curr_lane.lane_points.size() )
    {
        Pose2D point;
        point.x     = curr_lane.lane_points[first_pt + count].x;
        point.y     = curr_lane.lane_points[first_pt + count].y;
        point.theta = curr_lane.lane_points[first_pt + count].theta;

        points.push_back(point);
        count++;
    }

    return points;
}

Eigen::Spline3d NonholonomicAStar::getSpline( const std::vector<Pose2D>& points )
{
    Eigen::Vector3d spline_points( points.size() );

    for (std::uint32_t i = 0; i < points.size(); ++i)
    {
        spline_points(0, i) = points[i].x;
        spline_points(1, i) = points[i].y;
        spline_points(2, i) = points[i].theta;
    }

    return Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(spline_points, 3);
}

}
