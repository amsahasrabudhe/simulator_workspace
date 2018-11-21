/// @file This file contains class to hold 2D pose of the vehicle

#define toDegrees 57.2957795131
#define toRadians 0.01745329251

namespace mp
{

class Pose2D
{
    public:     /// functions
        Pose2D(const double& _x=0.0, const double& _y=0.0, const double& _theta=0.0):
            x(_x),
            y(_y),
            theta(_theta)
        {

        }

    public:     /// variables

        double x;
        double y;

        double theta;
};

}
