
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
{}

void NonholonomicAStar::initialize()
{
    // Find current lane
    localizeOnRoad();

    // Set current lane as desired lane on initialize
    m_overall_info->mp_info.desired_lane = m_overall_info->mp_info.current_lane;
}

void NonholonomicAStar::planPath()
{
    /// Get nearest lane point and lane id
    localizeOnRoad();

    if (m_planner_failed == false)
    {
        ros::Time start_time = ros::Time::now();

        // Clear contents of all vectors and queues from previous cycle
        std::priority_queue<Node, std::vector<Node>, CompareNodeCost> priority_queue;
        std::map<std::uint64_t, Node> all_nodes;
        m_open_list.clear();
        m_closed_list.clear();

        std::uint64_t node_count{1U};

        // Create node for the current state of ego vehicle
        const mp::EgoVehicle& ego = *(m_overall_info->ego_state);
        Node curr_node(node_count, ego.pose.x_m, ego.pose.y_m, ego.pose.heading_rad, ego.steering_rad, ego.vel_mps, ego.accel_mpss);

        Node start{curr_node};

        all_nodes[node_count] = curr_node;
        priority_queue.push(curr_node);

        double child_node_gen_time{0.0};
        double cost_calc_time{0.0};
        double child_node_mgmt_time{0.0};

        while ( priority_queue.empty() == false )
        {
            // Cancel path planning if its taking long time
            if ((ros::Time::now() - start_time).toSec() > m_cfg.planning_runtime_thresh_s)
            {
                std::cout<<"\nStopping because planner took forever : "<<(ros::Time::now() - start_time).toSec()<<std::endl;
                // m_planner_failed = true;
                break;
            }

            ros::Time before_child_nodes = ros::Time::now();    // Timing calcs

            // Get topmost node
            curr_node = priority_queue.top();
            priority_queue.pop();

            // Exit the planning step if the required distance is covered
            if (curr_node.distFrom(start) > (m_cfg.planned_path_time_s * start.vel_mps))
            {
                std::cout<<"Exiting because desired goal achieved : "<<curr_node.distFrom(start)<<std::endl;
                break;
            }

            std::vector<mp::Node> child_nodes = mp::getChildNodes(curr_node, m_cfg);

            ros::Time after_child_nodes = ros::Time::now();
            child_node_gen_time += (after_child_nodes-before_child_nodes).toSec();
            
            // Calculate cost for child node
            serial_mp::calculateCost(child_nodes, m_cfg, m_overall_info);

            ros::Time after_cost_calcs = ros::Time::now();
            cost_calc_time += (after_cost_calcs-after_child_nodes).toSec();

            const mp::LaneInfo& lane = m_overall_info->road_info.lanes[m_overall_info->mp_info.desired_lane];
            const double lane_center_y = lane.lane_points[m_overall_info->nearest_lane_point_with_index.first].y_m;

            // Add generated child nodes in priority queue
            bool obstacle_in_lane = false;
            for (auto& node : m_overall_info->mp_info.curr_eval_nodes)
            {
                // Use a new node for further analysis only if its safe
                if ( node.not_on_road == false && node.is_colliding == false )
                {
                    ++node_count;
                    node.node_index = node_count;
                    all_nodes[node_count] = node;

                    priority_queue.push(node);
                }
                else if (node.is_colliding == true)
                {
                    // TODO : Use offset from lane near the node's pose instead of this hack!!! 
                    // Check if obstacle is in our way
                    if (std::fabs(node.pose.y_m - lane_center_y) < 0.2)
                    {
                        obstacle_in_lane = true;
                    }
                }
            }

            // TODO: This crude logic doesn't work if there are obstacles in the adjacent lane
            // Change desired lane if obstacle found
            if (obstacle_in_lane == true)
            {
                const std::size_t total_lanes = m_overall_info->road_info.num_lanes;
                const std::size_t curr_lane_id = m_overall_info->mp_info.current_lane;

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
            }

            ros::Time after_child_nodes_management = ros::Time::now();
            child_node_mgmt_time += (after_child_nodes_management-after_cost_calcs).toSec();
        }
        
        if (m_planner_failed == false)
        {
            m_overall_info->mp_info.planned_path.clear();
            while ((all_nodes.empty() == false) && (curr_node.parent_index > 1U))
            {
                m_overall_info->mp_info.planned_path.push_back(curr_node);
                curr_node = all_nodes[curr_node.parent_index];
                // std::cout<<"Parent index: "<<curr_node.parent_index<<std::endl;
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
            m_overall_info->mp_info.curr_best_node.accel_mpss = m_cfg.braking_accel_mpss;
        }

        ros::Time end_time = ros::Time::now();

        std::cout<<"\nChild node generation : "<<child_node_gen_time<<std::endl;
        std::cout<<"Cost calculations : "<<cost_calc_time<<std::endl;
        std::cout<<"Child node management : "<<child_node_mgmt_time<<std::endl;
        std::cout<<"Plan Path Execution time : "<<(end_time-start_time).toSec()<<std::endl;
    }
}

void NonholonomicAStar::addToOpenList (const Node& /*node*/)
{

}

void NonholonomicAStar::addToClosedList (const Node& /*node*/)
{

}

void NonholonomicAStar::localizeOnRoad()
{
    std::size_t curr_lane = 0;
    std::pair<std::uint32_t, Pose2D> nearest_lane_point;

    // TODO: Optimize localizeOnRoad function to find nearest point

    double leastDist = 9999.0;
    for (std::size_t i = 0; i < m_overall_info->road_info.num_lanes; ++i)
    {
        LaneInfo lane = m_overall_info->road_info.lanes[i];
        for (std::size_t j = m_overall_info->nearest_lane_point_with_index.first; j < lane.lane_points.size(); ++j)
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

}