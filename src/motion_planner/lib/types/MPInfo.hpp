/// @file This file contains class to hold all important info required by MP

#ifndef MP_INFO_HPP
#define MP_INFO_HPP

#include <boost/optional.hpp>
#include <cstdint>
#include <memory>
#include <vector>

#include "lib/algorithm/astar/Node.hpp"

namespace mp
{

class MPInfo
{
    public:     /// functions
        MPInfo():
            current_lane(0),
            desired_lane(0),
            lane_offset(0.0)
        {
        }

    public:     /// variables

        /// Feedback and planning related variables for ego vehicle
        std::size_t current_lane;
        std::size_t desired_lane;
        double      lane_offset;

        /// Safety variables
        bool        within_road_boaundaries;
        bool        collision_state;

        /// Algorithm variables
        std::vector<Node> curr_eval_nodes;      /// Child nodes evaluated in the latest cycle
        Node              curr_best_node;       /// Control values used for the current update cycle
};

}

#endif //MP_INFO_HPP
