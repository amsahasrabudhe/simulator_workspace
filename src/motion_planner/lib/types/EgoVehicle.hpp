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
        EgoVehicle() :
            Vehicle()
        {

        }

    public:     /// variables

        bool placeholder_var;
};

}

#endif //EGO_VEHICLE_HPP
