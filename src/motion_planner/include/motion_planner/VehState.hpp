/// @file This file contains class to hold the current state of the vehicle (ego or traffic)

#include "Pose2D.hpp"
#include <cstdint>

namespace mp
{

class VehState
{
    public:     /// functions
        VehState():
            pose(),
            steering(0.0),
            vel(0.0),
            accel(0.0),
            desired_lane(0),
            lane_offset(0.0)
        {

        }

    public:     /// variables

        /// Kinematics related variables
        Pose2D  pose;
        double  steering;
        double  vel;
        double  accel;

        /// Feedback and planning related variables
        std::uint8_t    desired_lane;
        double          lane_offset;
};

}
