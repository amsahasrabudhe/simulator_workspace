/// @file This file contains boost geometry related functions

#ifndef BOOST_GEOMETRY_HELPER_FUNCTIONS_HPP
#define BOOST_GEOMETRY_HELPER_FUNCTIONS_HPP

#include <boost/assign.hpp>

#include <lib/types/RoadInfo.hpp>
#include <lib/types/Vehicle.hpp>

namespace mp
{

namespace geometry
{

inline BoostPointList getRoadPolygonPoints( const RoadInfo& road_info )
{
    BoostPolygon    road_polygon;
    BoostPointList  road_polygon_points;

    // Get leftmost lane edge
    LaneInfo leftmost_lane = road_info.lanes[0];
    double leftmost_lane_half_width = leftmost_lane.lane_width / 2;

    for ( std::vector<Pose2D>::const_iterator iter = leftmost_lane.lane_points.begin();
          iter != leftmost_lane.lane_points.end(); ++iter )
    {
        double x = leftmost_lane_half_width * cos(iter->heading_rad + M_PI_2) + iter->x_m;
        double y = leftmost_lane_half_width * sin(iter->heading_rad + M_PI_2) + iter->y_m;

        road_polygon_points.push_back( BoostPoint(x, y) );
    }

    // Get rightmost lane edge
    LaneInfo rightmost_lane = road_info.lanes[ road_info.num_lanes - 1 ];
    double rightmost_lane_half_width = rightmost_lane.lane_width / 2;

    for ( std::vector<Pose2D>::const_reverse_iterator iter = rightmost_lane.lane_points.rbegin();
          iter != rightmost_lane.lane_points.rend(); ++iter )
    {
        double x = rightmost_lane_half_width * cos(iter->heading_rad - M_PI_2) + iter->x_m;
        double y = rightmost_lane_half_width * sin(iter->heading_rad - M_PI_2) + iter->y_m;

        road_polygon_points.push_back( BoostPoint(x, y) );
    }

    road_polygon_points.push_back( BoostPoint( leftmost_lane.lane_points[0].x_m, leftmost_lane.lane_points[0].y_m ) );

    return road_polygon_points;
}

inline BoostPolygon getRoadPolygon( const RoadInfo& road_info )
{
    BoostPointList road_polygon_points = getRoadPolygonPoints(road_info);

    // Assign road edge points to road polygon
    BoostPolygon road_polygon;
    boost::geometry::assign_points(road_polygon, road_polygon_points);

    return road_polygon;
}

inline BoostPointList getVehiclePolygonPoints(const Vehicle& vehicle)
{
    BoostPointList veh_polygon_points;

    Pose2D vehicle_pose = vehicle.pose;
    double yaw = vehicle_pose.heading_rad;

    const double front      = 0.5 * (vehicle.length + vehicle.wheel_base);
    const double rear       = 0.5 * (vehicle.length - vehicle.wheel_base);
    const double half_width = 0.5 * vehicle.width;

    Pose2D front_left = Pose2D(cos(yaw), sin(yaw))*front + Pose2D(cos(yaw + M_PI_2), sin(yaw + M_PI_2))*half_width + vehicle.pose;
    Pose2D front_right = Pose2D(cos(yaw), sin(yaw))*front + Pose2D(cos(yaw - M_PI_2), sin(yaw - M_PI_2))*half_width + vehicle.pose;
    Pose2D rear_left = Pose2D(cos(yaw + M_PI_2), sin(yaw + M_PI_2))*half_width - Pose2D(cos(yaw), sin(yaw))*rear + vehicle.pose;
    Pose2D rear_right = Pose2D(cos(yaw - M_PI_2), sin(yaw - M_PI_2))*half_width - Pose2D(cos(yaw), sin(yaw))*rear + vehicle.pose;

    veh_polygon_points.push_back( BoostPoint(rear_left.x_m, rear_left.y_m) );
    veh_polygon_points.push_back( BoostPoint(front_left.x_m, front_left.y_m) );
    veh_polygon_points.push_back( BoostPoint(front_right.x_m, front_right.y_m) );
    veh_polygon_points.push_back( BoostPoint(rear_right.x_m, rear_right.y_m) );
    veh_polygon_points.push_back( BoostPoint(rear_left.x_m, rear_left.y_m) );

    return veh_polygon_points;
}

inline BoostPolygon getVehiclePolygonFromPoints( const BoostPointList& point_list )
{
    // Assign vehicle corner points to vehicle polygon
    BoostPolygon vehicle_polygon;
    boost::geometry::assign_points(vehicle_polygon, point_list);

    return vehicle_polygon;
}

}

}

#endif //BOOST_GEOMETRY_HELPER_FUNCTIONS
