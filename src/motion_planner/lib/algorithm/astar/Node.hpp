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
        Node(const double& x = 0.0, const double& y = 0.0, const double& theta = 0.0,
             const double& steering = 0.0, const double& vel = 0.0, const double& accel = 0.0):
            pose(Pose2D(x, y, theta)),
            steering(steering),
            vel(vel),
            accel(accel),
            gx(0.0),
            hx(0.0),
            parent(nullptr)
        {

        }

        void setParent(Node* parent)
        {
            this->parent = parent;
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
