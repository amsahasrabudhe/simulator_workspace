/// @file This file contains helper functions for motion planning algorithm

#ifndef MOTION_PLANNING_HELPER_FUNCTIONS_HPP
#define MOTION_PLANNING_HELPER_FUNCTIONS_HPP

#include <lib/algorithm/astar/Node.hpp>
#include <lib/configs/PlannerConfig.hpp>

namespace mp
{

std::vector<Node> getChildNodes(Node parent, const PlannerConfig& config)
{
    std::vector<Node> children;

    for (double steering_change = -15; steering_change <= 15; steering_change += 2.5)
    {
        for (double accel = -0.1; accel <= 0.1; accel += 0.05)
        {
            mp::Node child;

            double dt = config.child_node_dt;    ///< seconds

            double curr_theta = parent.pose.heading;
            double curr_vel = parent.vel;

            /// Add steering change to update steering angle for each child node
            double curr_steering = parent.steering + steering_change*toRadians;
            child.steering = curr_steering;

            if (child.steering > config.max_steering_rad)
                child.steering = config.max_steering_rad;

            double beta = dt * curr_vel * std::tan(curr_steering) / config.wheel_base;
            double R = dt * curr_vel / beta;

            if (std::fabs(beta) > 0.001)
            {
                child.pose.x = parent.pose.x + (std::sin(curr_theta+beta) - std::sin(curr_theta))*R;
                child.pose.y = parent.pose.y + (std::cos(curr_theta) - std::cos(curr_theta+beta))*R;
                child.pose.heading = fmod ((curr_theta+beta), 2*M_PI);
            }
            else
            {
                child.pose.x = parent.pose.x + dt * curr_vel * std::cos(curr_theta);
                child.pose.y = parent.pose.y + dt * curr_vel * std::sin(curr_theta);
                child.pose.heading = fmod ((curr_theta+beta), 2*M_PI);
            }

            child.vel = parent.vel + dt * parent.accel;
            child.accel = accel;

//            const double gx = child.distFrom(parent);

            child.setParentIndex(parent.node_index);

            children.push_back( child );
        }
    }

    return children;
}

}

#endif //MOTION_PLANNING_HELPER_FUNCTIONS
