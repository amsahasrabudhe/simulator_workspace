/// @file This file contains class to hold the overall information required for motion planning of the ego vehicle

#ifndef OVERALL_INFO_HPP
#define OVERALL_INFO_HPP

#include "EgoVehicle.hpp"
#include "RoadInfo.hpp"
#include "MPInfo.hpp"

#include <boost/optional.hpp>
#include <cstdint>
#include <memory>
#include <vector>

namespace mp
{

class OverallInfo
{
    public:     /// functions
        OverallInfo():
            num_traffic_vehicles(0),
            nearest_lane_point_with_index( std::make_pair(0, Pose2D()) )
        {
            ego_state = std::make_shared<EgoVehicle>();
        }

    public:     /// variables

        /// Ego vehicle related information
        std::shared_ptr<EgoVehicle>         ego_state;

        /// Road related information
        RoadInfo                            road_info;

        /// Traffic related info
        std::size_t                         num_traffic_vehicles;
        std::vector<Vehicle>                traffic;

        /// Localization related info
        std::pair<std::uint32_t, Pose2D>    nearest_lane_point_with_index;   ///< Nearest lane point with index
        std::vector<Pose2D>                 curr_poly_lanepoints;            ///< Current lane points used for poly fit

        /// Motion Planning related info
        MPInfo                              mp_info;
};

}

#endif
