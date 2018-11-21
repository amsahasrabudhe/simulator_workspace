#include "motion_planner/Planner.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");

    ros::NodeHandle nh;

    mp::Planner planner(nh);
    planner.init();

    ros::spin();
    return 0;
}
