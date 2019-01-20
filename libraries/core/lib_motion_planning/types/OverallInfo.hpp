/// @file This file contains class to hold the overall information required for motion planning of the ego vehicle

#include "EgoVehicle.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace mp
{

class OverallInfo
{
    public:     /// functions
        OverallInfo():
            num_traffic_vehicles(0)
        {
            ego_state = std::make_shared<EgoVehicle>();
        }

    public:     /// variables

        /// Ego vehicle related information
        std::shared_ptr<EgoVehicle> ego_state;

        /// Traffic related info
        std::size_t             num_traffic_vehicles;
        std::vector<Vehicle>    traffic;
};

}
