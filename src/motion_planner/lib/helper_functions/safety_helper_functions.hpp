/// @file This file contains functions to calculate safety of current state

#ifndef SAFETY_HELPER_FUNCTIONS_HPP
#define SAFETY_HELPER_FUNCTIONS_HPP

#include <cuda.h>
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <string>

#include "boost_geometry_helper_functions.hpp"

namespace mp
{

namespace safety
{

__host__ __device__ bool isWithinRoadBoundaries(const BoostPolygon& ego_polygon, const RoadInfo& road_info)
{
    if( boost::geometry::within(ego_polygon, road_info.road_polygon) )
    {
        return true;
    }

    return false;
}

__host__ __device__ bool isCollisionState(const BoostPolygon& ego_polygon, const Vehicle& traffic_veh_state)
{
    BoostPolygon traffic_veh_polygon = geometry::getVehiclePolygonFromPoints(traffic_veh_state.polygon_points);

    if ( boost::geometry::intersects(ego_polygon, traffic_veh_polygon) )
    {
        return true;
    }

    return false;
}

}

}

#endif //SAFETY_HELPER_FUNCTIONS
