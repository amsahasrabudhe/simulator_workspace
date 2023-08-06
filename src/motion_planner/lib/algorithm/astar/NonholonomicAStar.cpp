
#include "NonholonomicAStar.hpp"
#include "lib/algorithm/serial_cost_calculations.hpp"
#include "lib/helper_functions/motion_planning_helper_functions.hpp"

namespace mp
{

NonholonomicAStar::NonholonomicAStar(const ros::NodeHandle& nh, const std::shared_ptr<OverallInfo>& overall_info, const PlannerConfig& cfg):
    m_nh(nh),
    m_overall_info(overall_info),
    m_cfg(cfg),
    m_planner_failed(false)
{

}

void NonholonomicAStar::initialize()
{
    // Find current lane
    localize( m_overall_info->mp_info.current_lane, m_overall_info->nearest_lane_point_with_index.first );

    // Set currrent lane as desired lane on initialize
    m_overall_info->mp_info.desired_lane = m_overall_info->mp_info.current_lane;

    m_plan_path_timer = m_nh.createTimer(ros::Duration(m_cfg.plan_path_time_s), &NonholonomicAStar::planPath, this);
    m_plan_path_timer.start();
}

void NonholonomicAStar::update()
{
    // Get Lane points based on localization information
    m_overall_info->curr_poly_lanepoints = getLanePointsForPolyFit();

    // Fit spline for given lane points
//    m_overall_info->lane_center_spline = getSpline( m_overall_info->curr_poly_lanepoints );

}

void NonholonomicAStar::planPath(const ros::TimerEvent& /*event*/)
{
    if (m_planner_failed == false)
    {
        ros::Time start_time = ros::Time::now();

        // Clear contents of all vectors and queues from previous cycle
        std::priority_queue<Node, std::vector<Node>, CompareNodeCost> priority_queue;
        std::map<uint, Node> all_nodes;
        m_open_list.clear();
        m_closed_list.clear();

        uint node_num = 1;

        // Create node for the current state of ego vehicle
        mp::EgoVehicle ego = *(m_overall_info->ego_state);
        Node curr_node(node_num, ego.pose.x, ego.pose.y, ego.pose.heading, ego.steering, ego.vel, ego.accel);

        Node start = curr_node;

        all_nodes[node_num] = curr_node;
        priority_queue.push(curr_node);

        uint count  = 0;
        while ( !priority_queue.empty() )
        {
            // Cancel path planning if its taking long time
            if ((ros::Time::now() - start_time).toSec() > m_cfg.plan_path_time_s)
            {
                m_planner_failed = true;
                break;
            }

            ros::Time before_child_nodes = ros::Time::now();

            std::vector<mp::Node> total_child_nodes;

            bool dist_covered = false;
            uint i = 0;
            while (i < 15 && !priority_queue.empty())
            {
                // Get topmost node
                curr_node = priority_queue.top();
                priority_queue.pop();

                if (curr_node.distFrom(start) > m_cfg.dist_to_goal)
                {
                    dist_covered = true;
                    break;
                }

                // Generate child node
                std::vector<mp::Node> child_nodes = mp::getChildNodes(curr_node, m_cfg);

                for (std::size_t num = 0; num < child_nodes.size (); ++num)
                {
                    total_child_nodes.push_back( child_nodes.at(num) );
                }

                i++;
            }

            if (dist_covered)
                break;

            ros::Time after_child_nodes = ros::Time::now();

            std::cout<<"Child node generation : "<<(after_child_nodes-before_child_nodes).toSec()<<std::endl;

            // Calculate cost for child node
            serial_mp::calculateCost(total_child_nodes, m_cfg, m_overall_info);

            ros::Time after_cost_calcs = ros::Time::now();

            bool obstacle_found = false;

            const mp::LaneInfo& lane = m_overall_info->road_info.lanes.at( m_overall_info->mp_info.desired_lane );
            const double lane_center_y = lane.lane_points.at( m_overall_info->nearest_lane_point_with_index.first ).y;

            // Add generated child nodes in priority queue
            for (auto& node : m_overall_info->mp_info.curr_eval_nodes)
            {
                // Check if the node is safe
                if ( node.not_on_road == false && node.is_colliding == false )
                {
                    ++node_num;
                    node.node_index = node_num;
                    all_nodes[node_num] = node;

                    priority_queue.push (node);
                }
                else if (node.is_colliding)
                {
                    // TODO : Use offset from lane near the node's pose instead of this hack!!! 
                    
                    // Check if obstacle is in our way
                    if (fabs(node.pose.y - lane_center_y) < 0.2)
                        obstacle_found = true;
                    //break;    // Not sure if break is required
                }
            }

            // Change desired lane if obstacle found
            if (obstacle_found)
            {
                std::size_t total_lanes = m_overall_info->road_info.num_lanes;
                std::size_t curr_lane_id = m_overall_info->mp_info.current_lane;

                if (curr_lane_id == (total_lanes - 1) )
                {
                    // Lane change left if no lane is available on the right side
                    m_overall_info->mp_info.desired_lane = curr_lane_id - 1;
                }
                else if (curr_lane_id == 0)
                {
                    // Lane change right if no lane is available on the left side
                    m_overall_info->mp_info.desired_lane = curr_lane_id + 1;
                }
                else
                {
                    m_overall_info->mp_info.desired_lane = curr_lane_id - 1;
                }
                
                //break;    // Not sure if break is required
            }


            ros::Time after_child_nodes_management = ros::Time::now();
            std::cout<<"Child node management : "<<(after_child_nodes_management-after_cost_calcs).toSec()<<std::endl;

            count++;
        }

        if (m_planner_failed == false)
        {
            m_overall_info->mp_info.planned_path.clear ();
            while (all_nodes.size() > 1 && curr_node.parent_index != 1)
            {
                m_overall_info->mp_info.planned_path.push_back(curr_node);
                curr_node = all_nodes[curr_node.parent_index];
            }

            std::reverse(m_overall_info->mp_info.planned_path.begin(),
                         m_overall_info->mp_info.planned_path.end());

            // Save the current best node for execution
            m_overall_info->mp_info.curr_best_node = curr_node;
        }

        // Brake if path cannot be planned
        if ( (m_overall_info->mp_info.planned_path.empty()) ||
             (m_planner_failed == true) )
        {
            m_planner_failed = true;
            m_overall_info->mp_info.curr_best_node = start;
            m_overall_info->mp_info.curr_best_node.accel = m_cfg.braking_accel;
        }

        ros::Time end_time = ros::Time::now();
        std::cout<<"\nPlan Path Execution time : "<<(end_time-start_time).toSec()<<std::endl;
    }
}

void NonholonomicAStar::addToOpenList (const Node& /*node*/)
{

}

void NonholonomicAStar::addToClosedList (const Node& /*node*/)
{

}

void NonholonomicAStar::localize(const std::size_t /*known_current_lane*/, const std::size_t known_nearest_lane_point_index)
{
    std::size_t curr_lane = 0;
    std::pair<std::uint32_t, Pose2D> nearest_lane_point;

    // TODO: Optimize localize function to find nearest point

    double leastDist = 9999.0;
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

    LaneInfo curr_lane = m_overall_info->road_info.lanes[m_overall_info->mp_info.desired_lane];
    while (count < m_cfg.poly_fit_min_lane_points && (first_pt + count) <= curr_lane.lane_points.size() )
    {
        Pose2D point;
        point.x     = curr_lane.lane_points[first_pt + count].x;
        point.y     = curr_lane.lane_points[first_pt + count].y;
        point.heading = curr_lane.lane_points[first_pt + count].heading;

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
        spline_points(2, i) = points[i].heading;
    }

    return Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(spline_points, 3);
}

}
