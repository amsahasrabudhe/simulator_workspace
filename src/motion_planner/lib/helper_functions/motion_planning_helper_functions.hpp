/// @file This file contains helper functions for motion planning algorithm

#ifndef MOTION_PLANNING_HELPER_FUNCTIONS_HPP
#define MOTION_PLANNING_HELPER_FUNCTIONS_HPP

#include <lib/algorithm/astar/Node.hpp>
#include <lib/configs/PlannerConfig.hpp>

namespace mp
{

inline std::vector<Node> getChildNodes(const Node& parent, const PlannerConfig& config)
{
    std::vector<Node> children;

    const double max_accel_change_mpss = config.max_jerk_mpsss * config.child_node_dt;

    for (double steering_change_deg = -10.0; steering_change_deg <= 10.0; steering_change_deg += 2.5)
    {
        for (double accel_mpss = -max_accel_change_mpss; accel_mpss <= max_accel_change_mpss; accel_mpss += 0.1)
        {
            mp::Node child;

            const double dt = config.child_node_dt;    ///< seconds

            const double curr_heading_rad = parent.pose.heading_rad;
            const double curr_vel_mps = parent.vel_mps;

            /// Add steering change to update steering angle for each child node
            const double new_steering_rad = parent.steering_rad + steering_change_deg*toRadians;
            
            // Clamp to max steering angle
            child.steering_rad = std::min(new_steering_rad, config.max_steering_rad);

            // Calculate the heading angle change and radius of curvature

            child.pose.x_m = parent.pose.x_m + dt * curr_vel_mps * std::cos(curr_heading_rad);
            child.pose.y_m = parent.pose.y_m + dt * curr_vel_mps * std::sin(curr_heading_rad);
            
            const double heading_change_rad = dt * curr_vel_mps * std::tan(new_steering_rad) / config.wheel_base;
            child.pose.heading_rad = std::fmod((curr_heading_rad + heading_change_rad), 2*M_PI);

            child.vel_mps = parent.vel_mps + dt * parent.accel_mpss;
            child.accel_mpss = accel_mpss;

            // const double gx = child.distFrom(parent);

            child.setParentIndex(parent.node_index);

            children.push_back(child);
        }
    }

    return children;
}

}

#endif //MOTION_PLANNING_HELPER_FUNCTIONS
