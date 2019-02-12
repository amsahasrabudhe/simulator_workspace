/// @file This file contains class to hold all important info required by MP

#ifndef MP_INFO_HPP
#define MP_INFO_HPP

#include <boost/optional.hpp>
#include <cstdint>
#include <memory>
#include <vector>

namespace mp
{

class MPInfo
{
    public:     /// functions
        MPInfo():
            current_lane(-1),
            desired_lane(0),
            lane_offset(0.0)
        {
        }

    public:     /// variables

        /// Feedback and planning related variables for ego vehicle
        std::uint8_t    current_lane;
        std::uint8_t    desired_lane;
        double          lane_offset;
};

}

#endif //MP_INFO_HPP
