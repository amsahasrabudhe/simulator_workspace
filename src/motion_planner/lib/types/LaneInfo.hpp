/// @file This file contains class to individual Lane related information

#ifndef LANE_INFO_HPP
#define LANE_INFO_HPP

#include "Pose2D.hpp"

#include <vector>
#include <cstdint>

namespace mp
{

class LaneInfo
{
    public:     /// functions
        LaneInfo():
            lane_id(-1),
            lane_width(0.0)
        {
            
        }

    public:     /// variables

        std::int32_t        lane_id;
        double              lane_width;

        std::vector<Pose2D> lane_points;
};

}

#endif