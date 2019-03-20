/// @file This file contains helper functions for motion planning algorithm

#ifndef MOTION_PLANNING_HELPER_FUNCTIONS_HPP
#define MOTION_PLANNING_HELPER_FUNCTIONS_HPP

#include "boost_geometry_helper_functions.hpp"
#include <lib/algorithm/astar/Node.hpp>
#include <lib/configs/PlannerConfig.hpp>

namespace mp
{

std::vector<Node> getChildNodes(Node* parent, const PlannerConfig& config)
{
    std::vector<Node> children;

    for (int steering_change = -15; steering_change <= 15; steering_change += 5)
    {
        mp::Node child;
        child.setParent(parent);

        double dt = 2;    ///< seconds

        double curr_theta = parent->pose.theta;
        double curr_vel = parent->vel;

        /// Add steering change to update steering angle for each child node
        double curr_steering = parent->steering + steering_change*toRadians;
        child.steering = curr_steering;

        double beta = dt * curr_vel * tan(curr_steering) / config.wheel_base;
        double R = dt * curr_vel / beta;

        if (fabs(beta) > 0.001)
        {
            child.pose.x = parent->pose.x + (sin(curr_theta+beta) - sin(curr_theta))*R;
            child.pose.y = parent->pose.y + (cos(curr_theta) - cos(curr_theta+beta))*R;
            child.pose.theta = fmod ((curr_theta+beta), 2*M_PI);
        }
        else
        {
            child.pose.x = parent->pose.x + dt * curr_vel * cos(curr_theta);
            child.pose.y = parent->pose.y + dt * curr_vel * sin(curr_theta);
            child.pose.theta = fmod ((curr_theta+beta), 2*M_PI);
        }

        child.vel += dt * parent->accel;

        // TODO Add accel change

        child.gx = parent->gx;

        children.push_back( child );
    }

    return children;
}

}

#endif //MOTION_PLANNING_HELPER_FUNCTIONS
