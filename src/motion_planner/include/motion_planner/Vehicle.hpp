/// @file This file contains class to hold the current state of the vehicle in general

#include "Pose2D.hpp"
#include <cstdint>

namespace mp
{

class Vehicle
{
    public:     /// functions
        Vehicle(const double& init_x=0.0, const double& init_y=0.0, const double& init_heading=0.0 , const double& init_vel=0.0 , const double& init_accel=0.0):
            pose(init_x, init_y, init_heading),
            steering(0.0),
            vel(init_vel),
            accel(init_accel)
        {

        }

        void setPose(const double& x=0.0, const double& y=0.0, const double& theta=0.0)
        {
            this->pose.x = x;
            this->pose.y = y;
            this->pose.theta = theta;
        }

        void setVel(const double& vel=0.0)
        {
            this->vel = vel;
        }

        void setAccel(const double& accel=0.0)
        {
            this->accel = accel;
        }

    public:     /// variables

        Pose2D  pose;

        double  steering;
        double  vel;
        double  accel;
};

}
