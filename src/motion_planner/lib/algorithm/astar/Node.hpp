/// @file This file contains class for node in the A-Star search tree

#ifndef NODE_HPP
#define NODE_HPP

#include <cstdint>

#include <lib/types/Pose2D.hpp>

namespace mp
{

struct Node
{
    public:     /// functions
        Node(const std::uint64_t index = 0,
             const double x = 0.0, const double y = 0.0, const double heading = 0.0,
             const double steering_rad = 0.0,
             const double vel_mps = 0.0,
             const double accel_mpss = 0.0):
            pose(Pose2D(x, y, heading)),
            steering_rad(steering_rad),
            vel_mps(vel_mps),
            accel_mpss(accel_mpss),
            node_index(index),
            parent_index(0)
        {

        }

        void setGx(const double gx)                     {this->gx = gx;}
        void setNodeIndex(const std::uint64_t index)    {this->node_index = index;}
        void setParentIndex(const std::uint64_t index)  {this->parent_index = index;}

        double distFrom(const Node& other)
        {
            const double dx_squared = (pose.x_m - other.pose.x_m) * (pose.x_m - other.pose.x_m);
            const double dy_squared = (pose.y_m - other.pose.y_m) * (pose.y_m - other.pose.y_m);
            return std::sqrt(dx_squared + dy_squared);
        }

        bool operator==(const Node& other) const
        {
            const bool nodes_equal = (pose == other.pose); //&& (steering_rad - other.steering_rad);
            return nodes_equal;
        }

    public:     /// variables

        Pose2D pose;

        double steering_rad{0.0};
        double vel_mps{0.0};
        double accel_mpss{0.0};

        double gx{0.0};
        double hx{0.0};

        bool   not_on_road{false};
        bool   is_colliding{false};

        std::uint64_t node_index{0U};
        std::uint64_t parent_index{0U};
};

class CompareNodeCost
{
public:

    // Comparator to arrange in increasing order of cost
    bool operator()(Node const& node1, Node const& node2)
    {
        return (node1.gx + node1.hx) > (node2.gx + node2.hx);
    }
};

}

#endif
