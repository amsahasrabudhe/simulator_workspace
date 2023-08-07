#include "serial_cost_calculations.hpp"

#include <cmath>
#include <iostream>

namespace serial_mp
{

double Angle2D(double x1, double y1, double x2, double y2)
{
    const double theta_1 = std::atan2(y1, x1);
    const double theta_2 = std::atan2(y2, x2);

    double dtheta = theta_2 - theta_1;

    while (dtheta > M_PI)
    {
        dtheta -= 2*M_PI;
    }

    while (dtheta < -M_PI)
    {
        dtheta += 2*M_PI;
    }

    return dtheta;
}

bool isPointInsidePolygon(const mp::Vec2D& point, const std::vector<mp::Vec2D>& polygon_pts)
{
    double angle = 0.0;
    for (uint i = 0; i < polygon_pts.size(); ++i)
    {
        mp::Vec2D p1, p2;
        p1.x_m = polygon_pts[i].x_m - point.x_m;
        p1.y_m = polygon_pts[i].y_m - point.y_m;

        p2.x_m = polygon_pts[(i+1)%polygon_pts.size()].x_m - point.x_m;
        p2.y_m = polygon_pts[(i+1)%polygon_pts.size()].y_m - point.y_m;

        angle += Angle2D(p1.x_m, p1.y_m, p2.x_m, p2.y_m);
    }

    return (std::fabs(angle) > M_PI);
}

bool isNodeInsideRoad(const std::vector<mp::Vec2D>& child_node_polygon_points, const std::vector<mp::Vec2D>& road_polygon)
{
    for (const auto& child_node_poly_pt : child_node_polygon_points)
    {
        const bool inside = isPointInsidePolygon(child_node_poly_pt, road_polygon);

        // Exit loop if even one point is outside the road polygon
        if (inside == false)
        {
            return false;
        }
    }

    return true;
}

bool nodeCollidesWithTraffic(const std::vector<mp::Vec2D>& child_node_polygon_points, const std::vector<std::vector<mp::Vec2D>>& traffic_polygons)
{
    for (const auto& traffic_poly : traffic_polygons)
    {
        for (const auto& child_node_poly_pt : child_node_polygon_points)
        {
            const bool is_colliding = isPointInsidePolygon(child_node_poly_pt, traffic_poly);
            if (is_colliding == true)
            {
                return true;
            }
        }
    }

    return false;
}

std::vector<mp::Vec2D> calculateVehiclePolygonPoints(const mp::Node& node, const mp::Vehicle& ego_state)
{
    const double yaw = node.pose.heading_rad;

    const double front      = 0.5 * (ego_state.length + ego_state.wheel_base);
    const double rear       = 0.5 * (ego_state.length - ego_state.wheel_base);
    const double half_width = 0.5 * ego_state.width;

    mp::Vec2D front_left;
    front_left.x_m = node.pose.x_m + std::cos(yaw)*front + std::cos(yaw + M_PI_2)*half_width;
    front_left.y_m = node.pose.y_m + std::sin(yaw)*front + std::sin(yaw + M_PI_2)*half_width;

    mp::Vec2D front_right;
    front_right.x_m = node.pose.x_m + std::cos(yaw)*front + std::cos(yaw - M_PI_2)*half_width;
    front_right.y_m = node.pose.y_m + std::sin(yaw)*front + std::sin(yaw - M_PI_2)*half_width;

    mp::Vec2D rear_left;
    rear_left.x_m = node.pose.x_m - std::cos(yaw)*rear + std::cos(yaw + M_PI_2)*half_width;
    rear_left.y_m = node.pose.y_m - std::sin(yaw)*rear + std::sin(yaw + M_PI_2)*half_width;

    mp::Vec2D rear_right;
    rear_right.x_m = node.pose.x_m - std::cos(yaw)*rear + std::cos(yaw - M_PI_2)*half_width;
    rear_right.y_m = node.pose.y_m - std::sin(yaw)*rear + std::sin(yaw - M_PI_2)*half_width;

    std::vector<mp::Vec2D> veh_polygon_points{rear_left, front_left, front_right, rear_right};
    return veh_polygon_points;
}

void calculate_cost(std::vector<mp::Node>& child_nodes, const std::vector<mp::Vec2D>& road_polygon, 
                    const std::vector<std::vector<mp::Vec2D>>& traffic_polygons,
                    const double lane_center_y, const double planned_path_time_s, const mp::Vehicle& ego_state)
{
    for (auto& child_node : child_nodes)
    {
        const std::vector<mp::Vec2D>& child_node_polygon_points = calculateVehiclePolygonPoints(child_node, ego_state);

        // Calculate cost of going off road
        const bool inside_road_boundary = isNodeInsideRoad(child_node_polygon_points, road_polygon);
        // std::cout<<"Node outside road: "<<inside_road_boundary<<std::endl;
        if (inside_road_boundary == false)
        {
            child_node.not_on_road = true;
            child_node.hx += 50.0;
        }

        // Calculate cost of collision
        const bool node_collides = nodeCollidesWithTraffic (child_node_polygon_points, traffic_polygons);
        // std::cout<<"Node collides with traffic: "<<node_collides<<std::endl;
        if (node_collides == true)
        {
            child_node.is_colliding = true;
            child_node.hx += 50.0;
        }

        // TODO: This heuristic doesn't makes sense
        // Calculate cost of lane offset
        child_node.hx += 50.0 * std::fabs(child_node.pose.y_m - lane_center_y);

        const double dx_squared = (child_node.pose.x_m - ego_state.pose.x_m) * (child_node.pose.x_m - ego_state.pose.x_m);
        const double dy_squared = (child_node.pose.y_m - ego_state.pose.y_m) * (child_node.pose.y_m - ego_state.pose.y_m);
        double dist_from_start = std::sqrt(dx_squared + dy_squared);

        // Distance from goal heuristic
        child_node.hx += (ego_state.vel_mps*planned_path_time_s) - dist_from_start;
    }
}

void calculateCost(std::vector<mp::Node>& child_nodes, const mp::PlannerConfig& config, const std::shared_ptr<mp::OverallInfo>& overall_info)
{
    // Convert road polygon to array
    const std::vector<mp::Vec2D> road_polygon = calculateRoadPolygon(overall_info->road_info);

    // Convert traffic polygons to array
    const std::vector<std::vector<mp::Vec2D>> traffic_polygons = calculateTrafficPolygons(overall_info->traffic);

    const mp::LaneInfo& lane = overall_info->road_info.lanes[overall_info->mp_info.desired_lane];
    const double lane_center_y = lane.lane_points[overall_info->nearest_lane_point_with_index.first].y_m;

    // Kernel call
    calculate_cost(child_nodes, road_polygon, traffic_polygons,
                   lane_center_y, config.planned_path_time_s, *overall_info->ego_state);

    // Store nodes evaluated in the recent cycle
    overall_info->mp_info.curr_eval_nodes.clear();
    for (const auto& child_node : child_nodes)
    {
        overall_info->mp_info.curr_eval_nodes.push_back (child_node);
    }
}

std::vector<mp::Vec2D> calculateRoadPolygon(const mp::RoadInfo& road_info)
{
    std::vector<mp::Vec2D> polygon_points;

    mp::LaneInfo leftmost_lane = road_info.lanes.front();
    double leftmost_lane_half_width = 0.5 * leftmost_lane.lane_width;

    for ( std::vector<mp::Pose2D>::const_iterator iter = leftmost_lane.lane_points.begin();
          iter != leftmost_lane.lane_points.end(); ++iter )
    {
        mp::Vec2D point;
        point.x_m = iter->x_m + leftmost_lane_half_width * std::cos(iter->heading_rad + M_PI_2);
        point.y_m = iter->y_m + leftmost_lane_half_width * std::sin(iter->heading_rad + M_PI_2);

        polygon_points.push_back(point);
    }

    // Get rightmost lane edge
    mp::LaneInfo rightmost_lane = road_info.lanes.back();
    double rightmost_lane_half_width = 0.5 * rightmost_lane.lane_width;

    for ( std::vector<mp::Pose2D>::const_reverse_iterator iter = rightmost_lane.lane_points.rbegin();
          iter != rightmost_lane.lane_points.rend(); ++iter )
    {
        mp::Vec2D point;
        point.x_m = iter->x_m + rightmost_lane_half_width * std::cos(iter->heading_rad - M_PI_2);
        point.y_m = iter->y_m + rightmost_lane_half_width * std::sin(iter->heading_rad - M_PI_2);

        polygon_points.push_back(point);
    }

    return polygon_points;
}

std::vector<std::vector<mp::Vec2D>> calculateTrafficPolygons(const std::vector<mp::Vehicle>& traffic)
{
    std::vector<std::vector<mp::Vec2D>> traffic_polygons;

    for (const auto& vehicle : traffic)
    {
        const double yaw = vehicle.pose.heading_rad;

        double half_length = 0.5 * vehicle.length;    // vehicle.length/2 (since simulator publishes midpoint as pose)
        double half_width = 0.5 * vehicle.width;

        mp::Vec2D front_left;
        front_left.x_m =  vehicle.pose.x_m + std::cos(yaw)*half_length + std::cos(yaw + M_PI_2)*half_width;
        front_left.y_m =  vehicle.pose.y_m + std::sin(yaw)*half_length + std::sin(yaw + M_PI_2)*half_width;

        mp::Vec2D front_right;
        front_right.x_m =  vehicle.pose.x_m + std::cos(yaw)*half_length + std::cos(yaw - M_PI_2)*half_width;
        front_right.y_m =  vehicle.pose.y_m + std::sin(yaw)*half_length + std::sin(yaw - M_PI_2)*half_width;

        mp::Vec2D rear_left;
        rear_left.x_m = vehicle.pose.x_m - std::cos(yaw)*half_length + std::cos(yaw + M_PI_2)*half_width;
        rear_left.y_m = vehicle.pose.y_m - std::sin(yaw)*half_length + std::sin(yaw + M_PI_2)*half_width;

        mp::Vec2D rear_right;
        rear_right.x_m = vehicle.pose.x_m - std::cos(yaw)*half_length + std::cos(yaw - M_PI_2)*half_width;
        rear_right.y_m = vehicle.pose.y_m - std::sin(yaw)*half_length + std::sin(yaw - M_PI_2)*half_width;

        std::vector<mp::Vec2D> curr_veh_polygon{rear_left, front_left, front_right, rear_right};
        traffic_polygons.push_back(std::move(curr_veh_polygon));
    }

    return traffic_polygons;
}


}
