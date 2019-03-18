/// @file This file contains boost geometry related functions

#ifndef BOOST_GEOMETRY_HELPER_FUNCTIONS_HPP
#define BOOST_GEOMETRY_HELPER_FUNCTIONS_HPP

#include <boost/assign.hpp>
#include <cuda.h>
#include <cuda_runtime.h>
#include <curand_kernel.h>

#include <lib/types/RoadInfo.hpp>
#include <lib/types/Vehicle.hpp>

namespace mp
{

namespace geometry
{

BoostPointList getRoadPolygonPoints( const RoadInfo& road_info )
{
    BoostPolygon    road_polygon;
    BoostPointList  road_polygon_points;

    // Get leftmost lane edge
    LaneInfo leftmost_lane = road_info.lanes[0];
    double leftmost_lane_half_width = leftmost_lane.lane_width / 2;

    for ( std::vector<Pose2D>::const_iterator iter = leftmost_lane.lane_points.begin();
          iter != leftmost_lane.lane_points.end(); ++iter )
    {
        double x = leftmost_lane_half_width * cos(iter->theta + M_PI_2) + iter->x;
        double y = leftmost_lane_half_width * sin(iter->theta + M_PI_2) + iter->y;

        road_polygon_points.push_back( BoostPoint(x, y) );
    }

    // Get rightmost lane edge
    LaneInfo rightmost_lane = road_info.lanes[ road_info.num_lanes - 1 ];
    double rightmost_lane_half_width = rightmost_lane.lane_width / 2;

    for ( std::vector<Pose2D>::const_reverse_iterator iter = rightmost_lane.lane_points.rbegin();
          iter != rightmost_lane.lane_points.rend(); ++iter )
    {
        double x = rightmost_lane_half_width * cos(iter->theta - M_PI_2) + iter->x;
        double y = rightmost_lane_half_width * sin(iter->theta - M_PI_2) + iter->y;

        road_polygon_points.push_back( BoostPoint(x, y) );
    }

    road_polygon_points.push_back( BoostPoint( leftmost_lane.lane_points[0].x, leftmost_lane.lane_points[0].y ) );

    return road_polygon_points;
}

BoostPolygon getRoadPolygon( const RoadInfo& road_info )
{
    BoostPointList road_polygon_points = getRoadPolygonPoints(road_info);

    // Assign road edge points to road polygon
    BoostPolygon road_polygon;
    boost::geometry::assign_points(road_polygon, road_polygon_points);

    return road_polygon;
}

BoostPointList getVehiclePolygonPoints( const Vehicle& vehicle )
{
    BoostPointList veh_polygon_points;

    Pose2D vehicle_pose = vehicle.pose;
    double yaw = vehicle_pose.theta;

    double front = 3.869;    // approx (vehicle.length/2 + vehicle.wheel_base/2)
    double rear  = 0.910;    // approx (vehicle.length/2 - vehicle.wheel_base/2)
    double width_by_2 = vehicle.width / 2;

    Pose2D front_left = Pose2D( cos(yaw), sin(yaw) )*front + Pose2D(cos(yaw + M_PI_2), sin(yaw + M_PI_2))*width_by_2 + vehicle.pose;
    Pose2D front_right = Pose2D( cos(yaw), sin(yaw) )*front + Pose2D(cos(yaw - M_PI_2), sin(yaw - M_PI_2))*width_by_2 + vehicle.pose;
    Pose2D rear_left = Pose2D(cos(yaw + M_PI_2), sin(yaw + M_PI_2))*width_by_2 - Pose2D( cos(yaw), sin(yaw) )*rear + vehicle.pose;
    Pose2D rear_right = Pose2D(cos(yaw - M_PI_2), sin(yaw - M_PI_2))*width_by_2 - Pose2D( cos(yaw), sin(yaw) )*rear + vehicle.pose;

    veh_polygon_points.push_back( BoostPoint(rear_left.x, rear_left.y) );
    veh_polygon_points.push_back( BoostPoint(front_left.x, front_left.y) );
    veh_polygon_points.push_back( BoostPoint(front_right.x, front_right.y) );
    veh_polygon_points.push_back( BoostPoint(rear_right.x, rear_right.y) );
    veh_polygon_points.push_back( BoostPoint(rear_left.x, rear_left.y) );

    return veh_polygon_points;
}

BoostPolygon getVehiclePolygonFromPoints( const BoostPointList& point_list )
{
    // Assign vehicle corner points to vehicle polygon
    BoostPolygon vehicle_polygon;
    boost::geometry::assign_points(vehicle_polygon, point_list);

    return vehicle_polygon;
}

}

}

#endif //BOOST_GEOMETRY_HELPER_FUNCTIONS
