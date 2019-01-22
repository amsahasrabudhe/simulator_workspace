#include "motion_planner/PlannerROSInterface.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");

    ros::NodeHandle nh;

    mp::PlannerROSInterface planner_ros_interface(nh);
    planner_ros_interface.init();

    ros::spin();
    return 0;
}
