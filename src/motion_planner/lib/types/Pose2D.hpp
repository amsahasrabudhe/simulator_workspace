/// @file This file contains class to hold 2D pose of the vehicle

#ifndef POSE_2D_HPP
#define POSE_2D_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <math.h>
#include <string>
#include <sstream>

#define toDegrees 57.2957795131
#define toRadians 0.01745329251

namespace mp
{

using BoostPoint = boost::geometry::model::d2::point_xy<double>;
using BoostPointList = std::vector<BoostPoint>;
using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

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

        double distFrom(const Pose2D other)
        {
            return pow((pow((x - other.x), 2) + pow((y - other.y), 2)), 0.5);
        }

        BoostPoint getBoostPoint()
        {
            return BoostPoint(x, y);
        }

        std::string toString()
        {
            std::stringstream ss;
            ss << "X: "<< x<< " , Y: "<< y<< " , Theta: "<< theta;

            return ss.str();
        }

    public:     /// variables

        double x;
        double y;

        double theta;
};

}

#endif
