/// @file This file contains class to hold the current state of the vehicle in general

#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include "Pose2D.hpp"
#include <cstdint>

namespace mp
{

class Vehicle
{
    public:     /// functions
        Vehicle(const double& init_x=0.0, const double& init_y=0.0, const double& init_heading=0.0 , const double& init_vel=0.0 , const double& init_accel=0.0):
            pose(init_x, init_y, init_heading),
            vel(init_vel),
            accel(init_accel)
        {

        }

        void setPose(const double& x=0.0, const double& y=0.0, const double& heading=0.0)
        {
            this->pose.x = x;
            this->pose.y = y;
            this->pose.heading = heading;
        }

        void setVel(const double& vel=0.0)
        {
            this->vel = vel;
        }

        void setAccel(const double& accel=0.0)
        {
            this->accel = accel;
        }

        void setSteeringAngle(const double& steering_angle=0.0)
        {
            this->steering = steering_angle;
        }

    public:     /// variables

        std::uint8_t id{0};

        double length{4.97};
        double width{2.1};

        Pose2D pose;

        double steering{0.0};
        double vel;
        double accel;

        BoostPointList  polygon_points;
};

}

#endif
