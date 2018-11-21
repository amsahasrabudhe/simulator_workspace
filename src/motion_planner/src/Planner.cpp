/// @file

#include "motion_planner/Planner.hpp"

#include <math.h>
#include <simulator_msgs/VehState.h>

namespace mp
{

Planner::Planner(const ros::NodeHandle& nh):
    m_nh(nh)
{

}

void Planner::init()
{
    setupEgoVehicle();

    m_ego_state_pub = m_nh.advertise<simulator_msgs::VehState>(m_config.ego_veh_state_out_topic, 1, false);

    m_update_timer = m_nh.createTimer(ros::Duration(m_config.update_time_s), &Planner::update, this);
    m_update_timer.start();
}

void Planner::update(const ros::TimerEvent& event)
{
    publishEgoVehicleState();
}

void Planner::setupEgoVehicle()
{
    m_ego_state.setPose(90, 395, 0);
    m_ego_state.setVel(2);
    m_ego_state.setAccel(0.2);
}

void Planner::publishEgoVehicleState()
{
    m_ego_state.pose.x += m_ego_state.vel * cos( m_ego_state.pose.theta * toRadians );
    m_ego_state.pose.y += m_ego_state.vel * sin( m_ego_state.pose.theta * toRadians );

    simulator_msgs::VehState ros_ego_state;

    ros_ego_state.header.stamp  = ros::Time::now();

    ros_ego_state.pose.x        = m_ego_state.pose.x;
    ros_ego_state.pose.y        = m_ego_state.pose.y;
    ros_ego_state.pose.theta    = m_ego_state.pose.theta;

    m_ego_state_pub.publish( ros_ego_state );
}

}
