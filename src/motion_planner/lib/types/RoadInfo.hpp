/// @file This file contains class to hold drivable road related information

#ifndef ROAD_INFO_HPP
#define ROAD_INFO_HPP

#include "LaneInfo.hpp"

#include <cstdint>
#include <vector>

namespace mp
{

class RoadInfo
{
    public:     /// functions
        RoadInfo():
        	num_lanes(0)
        {

        }

    public:     /// variables

        std::uint8_t            num_lanes;      ///< Number of lanes on the road
        std::vector<LaneInfo>   lanes;          ///< Vector of Lanes

        BoostPointList          road_polygon_points;    ///< Boost point list representing road boundaries
        BoostPolygon            road_polygon;           ///< Boost polygon representing road boundaries
};

}

#endif
