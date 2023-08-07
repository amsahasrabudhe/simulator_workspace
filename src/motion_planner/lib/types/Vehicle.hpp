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
        Vehicle(const double init_x_m=0.0, const double init_y_m=0.0, const double init_heading=0.0, 
                const double init_vel_mps=0.0 , const double init_accel_mpss=0.0):
            pose(init_x_m, init_y_m, init_heading),
            vel_mps(init_vel_mps),
            accel_mpss(init_accel_mpss)
        {}

        void setPose(const double& x_m=0.0, const double& y_m=0.0, const double& heading_rad=0.0)
        {
            this->pose.x_m = x_m;
            this->pose.y_m = y_m;
            this->pose.heading_rad = heading_rad;
        }

        void setVel(const double& vel_mps=0.0)
        {
            this->vel_mps = vel_mps;
        }

        void setAccel(const double& accel_mpss=0.0)
        {
            this->accel_mpss = accel_mpss;
        }

        void setSteeringAngle(const double& steering_rad=0.0)
        {
            this->steering_rad = steering_rad;
        }

    public:     /// variables

        std::uint8_t id{0};

        double length{4.97};
        double wheel_base{2.81};
        double width{2.1};

        Pose2D pose;

        double steering_rad{0.0};
        double vel_mps;
        double accel_mpss;

        BoostPointList  polygon_points;
};

}

#endif
