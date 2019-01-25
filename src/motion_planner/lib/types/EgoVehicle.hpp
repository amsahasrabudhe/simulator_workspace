/// @file This file contains class to hold the current state of the vehicle (ego or traffic)

#ifndef EGO_VEHICLE_HPP
#define EGO_VEHICLE_HPP

#include "Vehicle.hpp"
#include <cstdint>

namespace mp
{

class EgoVehicle : public Vehicle
{
    public:     /// functions
        EgoVehicle():
            Vehicle(),
            desired_lane(0),
            lane_offset(0.0)
        {

        }

    public:     /// variables

        /// Feedback and planning related variables for ego vehicle
        std::uint8_t    desired_lane;
        double          lane_offset;
};

}

#endif //EGO_VEHICLE_HPP