#include "serial_cost_calculations.hpp"

namespace serial_mp
{

double Angle2D(double x1, double y1, double x2, double y2)
{
   double theta_1, theta_2;
   double dtheta;

   theta_1 = atan2(y1,x1);
   theta_2 = atan2(y2,x2);

   dtheta = theta_2 - theta_1;

   while (dtheta > M_PI)
      dtheta -= 2*M_PI;

   while (dtheta < -M_PI)
      dtheta += 2*M_PI;

   return(dtheta);
}

bool isPointInsidePolygon(const mp::Vec2D& point, const mp::Vec2D* polygon, const std::size_t polygon_point_count)
{
    double angle = 0.0;

    mp::Vec2D p1, p2;

    for (uint i = 0; i < polygon_point_count; ++i)
    {
        p1.x = polygon[i].x - point.x;
        p1.y = polygon[i].y - point.y;

        p2.x = polygon[(i+1)%polygon_point_count].x - point.x;
        p2.y = polygon[(i+1)%polygon_point_count].y - point.y;

        angle += Angle2D (p1.x, p1.y, p2.x, p2.y);
    }

    if (fabs(angle) < M_PI)
        return false;

    return true;
}

bool isNodeInsideRoad(const mp::Vec2D* child_node_polygon_points, const mp::Vec2D* road_polygon, const std::size_t road_polygon_point_count)
{
    bool inside = true;

    for (std::size_t i=0; i < 4; ++i)
    {
        inside = isPointInsidePolygon (child_node_polygon_points[i], road_polygon, road_polygon_point_count);

        // Exit loop if even one point is outside the road polygon
        if (!inside)
        {
            break;
        }
    }

    return inside;
}

bool nodeCollidesWithTraffic(const mp::Vec2D* child_node_polygon_points, const mp::Vec2D* traffic_polygons, const std::size_t traffic_veh_count)
{
    bool is_colliding = false;

    for (std::size_t veh_num = 0; veh_num < traffic_veh_count; ++veh_num)
    {
        for (std::size_t i = 0; i < 4; ++i)
        {
            is_colliding = isPointInsidePolygon(child_node_polygon_points[i],
                                                (traffic_polygons + 4*veh_num), 4);

            if (is_colliding)
                return true;
        }
    }

    return false;
}

void calculateVehiclePolygonPoints(const mp::Node& node, mp::Vec2D* veh_polygon_points)
{
    double yaw = node.pose.theta;

    double front      = 3.869;    // approx (vehicle.length/2 + vehicle.wheel_base/2)
    double rear       = 0.910;    // approx (vehicle.length/2 - vehicle.wheel_base/2)
    double width_by_2 = 1.05;

    mp::Vec2D front_left;
    front_left.x = cos(yaw)*front + cos(yaw + M_PI_2)*width_by_2 + node.pose.x;
    front_left.y = sin(yaw)*front + sin(yaw + M_PI_2)*width_by_2 + node.pose.y;

    mp::Vec2D front_right;
    front_right.x = cos(yaw)*front + cos(yaw - M_PI_2)*width_by_2 + node.pose.x;
    front_right.y = sin(yaw)*front + sin(yaw - M_PI_2)*width_by_2 + node.pose.y;

    mp::Vec2D rear_left;
    rear_left.x = - cos(yaw)*rear + cos(yaw + M_PI_2)*width_by_2 + node.pose.x;
    rear_left.y = - sin(yaw)*rear + sin(yaw + M_PI_2)*width_by_2 + node.pose.y;

    mp::Vec2D rear_right;
    rear_right.x = - cos(yaw)*rear + cos(yaw - M_PI_2)*width_by_2 + node.pose.x;
    rear_right.y = - sin(yaw)*rear + sin(yaw - M_PI_2)*width_by_2 + node.pose.y;

    veh_polygon_points[0] = rear_left;
    veh_polygon_points[1] = front_left;
    veh_polygon_points[2] = front_right;
    veh_polygon_points[3] = rear_right;
}

void calculate_cost(mp::Node* device_child_node_array, const std::size_t child_node_count,
                    mp::Vec2D* road_polygon, const std::size_t road_polygon_point_count,
                    mp::Vec2D* traffic_polygons, const std::size_t traffic_veh_count,
                    const double lane_center_y, const double dist_to_goal,
                    const double start_x, const double start_y)
{
    for (std::size_t curr_index = 0; curr_index < child_node_count; ++curr_index)
    {
        mp::Node* curr_child_node = &device_child_node_array[curr_index];

        mp::Vec2D child_node_polygon_points[4];
        calculateVehiclePolygonPoints( *curr_child_node, child_node_polygon_points );

        // Calculate cost of going off road
        bool inside_road_boundary = isNodeInsideRoad(child_node_polygon_points, road_polygon, road_polygon_point_count);

        if (!inside_road_boundary)
        {
            curr_child_node->not_on_road = true;
            curr_child_node->hx += 50;
        }

        // Calculate cost of collision
        bool node_collides = nodeCollidesWithTraffic (child_node_polygon_points, traffic_polygons, traffic_veh_count);

        if (node_collides)
        {
            curr_child_node->is_colliding = true;
            curr_child_node->hx += 50;
        }

        // Calculate cost of lane offset
        curr_child_node->hx += fabs(curr_child_node->pose.y - lane_center_y);

        double dist_sq = pow((curr_child_node->pose.x - start_x), 2) +
                pow((curr_child_node->pose.y - start_y), 2);

        double dist_from_start = pow(dist_sq, 0.5);

        // Distance from goal heuristic
        curr_child_node->hx += dist_to_goal - dist_from_start;
    }
}

void calculateCost(std::vector<mp::Node>& child_nodes, const mp::PlannerConfig& config, const std::shared_ptr<mp::OverallInfo>& overall_info)
{
    //convert to child node vector to array
    mp::Node* host_child_node_array = child_nodes.data();
    std::size_t totalChildNodes = child_nodes.size();

    // Convert road polygon to array
    std::vector<mp::Vec2D> road_polygon = calculateRoadPolygon (overall_info->road_info);
    mp::Vec2D* host_road_polygon = road_polygon.data();

    std::size_t totalRoadPolygonPoints = road_polygon.size();

    // Convert traffic polygons to array
    std::vector<mp::Vec2D> traffic_polygons = calculateTrafficPolygons (overall_info->traffic);

    mp::Vec2D* host_traffic_polygons_array = traffic_polygons.data();

    std::size_t total_traffic_vehicles = overall_info->traffic.size();

    const mp::LaneInfo& lane = overall_info->road_info.lanes.at( overall_info->mp_info.desired_lane );
    double lane_center_y = lane.lane_points.at( overall_info->nearest_lane_point_with_index.first ).y;

    // Kernel call
    calculate_cost (host_child_node_array, totalChildNodes,
                    host_road_polygon, totalRoadPolygonPoints,
                    host_traffic_polygons_array, total_traffic_vehicles,
                    lane_center_y, config.dist_to_goal,
                    overall_info->ego_state->pose.x, overall_info->ego_state->pose.y);

    // Store nodes evaluated in the recent cycle
    overall_info->mp_info.curr_eval_nodes.clear();
    for (uint i = 0; i < totalChildNodes; ++i)
    {
        overall_info->mp_info.curr_eval_nodes.push_back (host_child_node_array[i]);
    }
}

std::vector<mp::Vec2D> calculateRoadPolygon(const mp::RoadInfo& road_info)
{
    std::vector<mp::Vec2D> polygon_points;

    mp::LaneInfo leftmost_lane = road_info.lanes[0];
    double leftmost_lane_half_width = leftmost_lane.lane_width / 2;

    for ( std::vector<mp::Pose2D>::const_iterator iter = leftmost_lane.lane_points.begin();
          iter != leftmost_lane.lane_points.end(); ++iter )
    {
        mp::Vec2D point;
        point.x = leftmost_lane_half_width * cos(iter->theta + M_PI_2) + iter->x;
        point.y = leftmost_lane_half_width * sin(iter->theta + M_PI_2) + iter->y;

        polygon_points.push_back( point );
    }

    // Get rightmost lane edge
    mp::LaneInfo rightmost_lane = road_info.lanes[ road_info.num_lanes - 1 ];
    double rightmost_lane_half_width = rightmost_lane.lane_width / 2;

    for ( std::vector<mp::Pose2D>::const_reverse_iterator iter = rightmost_lane.lane_points.rbegin();
          iter != rightmost_lane.lane_points.rend(); ++iter )
    {
        mp::Vec2D point;
        point.x = rightmost_lane_half_width * cos(iter->theta - M_PI_2) + iter->x;
        point.y = rightmost_lane_half_width * sin(iter->theta - M_PI_2) + iter->y;

        polygon_points.push_back( point );
    }

    return polygon_points;
}

std::vector<mp::Vec2D> calculateTrafficPolygons(const std::vector<mp::Vehicle>& traffic)
{
    std::vector<mp::Vec2D> traffic_polygons;

    for (const auto& vehicle : traffic)
    {
        double yaw = vehicle.pose.theta;

        double front      = 2.389;    // vehicle.length/2 (since simulator publishes midpoint as pose)
        double rear       = 2.389;    // vehicle.length/2
        double width_by_2 = 1.05;

        mp::Vec2D front_left;
        front_left.x = cos(yaw)*front + cos(yaw + M_PI_2)*width_by_2 + vehicle.pose.x;
        front_left.y = sin(yaw)*front + sin(yaw + M_PI_2)*width_by_2 + vehicle.pose.y;

        mp::Vec2D front_right;
        front_right.x = cos(yaw)*front + cos(yaw - M_PI_2)*width_by_2 + vehicle.pose.x;
        front_right.y = sin(yaw)*front + sin(yaw - M_PI_2)*width_by_2 + vehicle.pose.y;

        mp::Vec2D rear_left;
        rear_left.x = - cos(yaw)*rear + cos(yaw + M_PI_2)*width_by_2 + vehicle.pose.x;
        rear_left.y = - sin(yaw)*rear + sin(yaw + M_PI_2)*width_by_2 + vehicle.pose.y;

        mp::Vec2D rear_right;
        rear_right.x = - cos(yaw)*rear + cos(yaw - M_PI_2)*width_by_2 + vehicle.pose.x;
        rear_right.y = - sin(yaw)*rear + sin(yaw - M_PI_2)*width_by_2 + vehicle.pose.y;

        traffic_polygons.push_back (rear_left);
        traffic_polygons.push_back (front_left);
        traffic_polygons.push_back (front_right);
        traffic_polygons.push_back (rear_right);
    }

    return traffic_polygons;
}


}
