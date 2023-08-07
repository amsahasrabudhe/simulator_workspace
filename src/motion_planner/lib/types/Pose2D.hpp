/// @file This file contains class to hold 2D pose of the vehicle

#ifndef POSE_2D_HPP
#define POSE_2D_HPP

#include "Vec2D.hpp"

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
        Pose2D(const double& _x_m=0.0, const double& _y_m=0.0, const double& _heading_rad=0.0):
            x_m(_x_m),
            y_m(_y_m),
            heading_rad(_heading_rad)
        {}

        Pose2D operator+ (const Pose2D& other)
        {
            return Pose2D(this->x_m+other.x_m, this->y_m+other.y_m, this->heading_rad+other.heading_rad);
        }

        Pose2D operator- (const Pose2D& other)
        {
            return Pose2D(this->x_m-other.x_m, this->y_m-other.y_m, this->heading_rad-other.heading_rad);
        }

        Pose2D operator* (const double& factor)
        {
            return Pose2D(this->x_m*factor, this->y_m*factor, this->heading_rad*factor);
        }

        Pose2D operator/ (const double& factor)
        {
            return Pose2D(this->x_m/factor, this->y_m/factor, this->heading_rad/factor);
        }

        bool operator== (const Pose2D& other) const
        {
            if (((x_m - other.x_m) < 0.5) &&
                ((y_m - other.y_m) < 0.5) &&
                ((heading_rad - other.heading_rad) < 0.001))
            {
                return true;
            }

            return false;
        }

        double distFrom(const Pose2D other)
        {
            const double dx_squared = (x_m - other.x_m) * (x_m - other.x_m);
            const double dy_squared = (y_m - other.y_m) * (y_m - other.y_m);

            return std::sqrt(dx_squared + dy_squared);
        }

        BoostPoint getBoostPoint()
        {
            return BoostPoint(x_m, y_m);
        }

        Vec2D getVec2d()
        {
            return Vec2D{x_m, y_m};
        }

        std::string toString()
        {
            std::stringstream ss;
            ss << "X(m): "<< x_m<< " , Y(m): "<< y_m<< " , Heading(rad): "<< heading_rad;

            return ss.str();
        }

    public:     /// variables

        double x_m;
        double y_m;

        double heading_rad;
};

}

#endif
