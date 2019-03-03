/// @file This file contains class for node in the A-Star search tree

#ifndef NODE_HPP
#define NODE_HPP

#include <cstdint>

#include <lib/types/Pose2D.hpp>

namespace mp
{

class Node
{
    public:     /// functions
        Node():
            steering(0.0),
            vel(0.0),
            accel(0.0),
            gx(0.0),
            hx(0.0),
            parent(nullptr)
        {

        }

    public:     /// variables

        Pose2D pose;

        double steering;
        double vel;
        double accel;

        double gx;
        double hx;

        Node   *parent;
};

}

#endif
