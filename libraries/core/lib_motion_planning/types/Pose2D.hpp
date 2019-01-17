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

        Pose2D operator+ (const Pose2D& other)
        {
            return Pose2D(this->x+other.x, this->y+other.y, this->theta+other.theta);
        }

        Pose2D operator- (const Pose2D& other)
        {
            return Pose2D(this->x-other.x, this->y-other.y, this->theta-other.theta);
        }

        Pose2D operator* (const double& factor)
        {
            return Pose2D(this->x*factor, this->y*factor, this->theta*factor);
        }

        Pose2D operator/ (const double& factor)
        {
            return Pose2D(this->x/factor, this->y/factor, this->theta/factor);
        }

    public:     /// variables

        double x;
        double y;

        double theta;
};

}
