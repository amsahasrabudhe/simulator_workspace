/// @file This file contains class to hold drivable road related information

#ifndef ROAD_INFO_HPP
#define ROAD_INFO_HPP

#include "LaneInfo.hpp"

#include <vector>
#include <cstdint>

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

        std::uint8_t            num_lanes;

        std::vector<LaneInfo>   lanes;
};

}

#endif