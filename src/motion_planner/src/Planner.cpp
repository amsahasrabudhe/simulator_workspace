/// @file

#include "motion_planner/Planner.hpp"

#include <math.h>
#include <simulator_msgs/VehState.h>

namespace mp
{

Planner::Planner(const ros::NodeHandle& nh):
    m_nh(nh),
    m_last_update_time(ros::Time::now())
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
    m_ego_state.setPose(6.326489009482704, 24.673307136982547, 0);
    m_ego_state.setVel(10.5);
    m_ego_state.setAccel(1);
    m_ego_state.setSteeringAngle(1);
}

void Planner::publishEgoVehicleState()
{
    ros::Time now = ros::Time::now();
    /// Calculate dt since last update in seconds
    double dt = now.toSec() - m_last_update_time.toSec();

    m_last_update_time = now;

    double angular_vel_deg_per_sec = (m_ego_state.vel * tan( m_ego_state.steering * toRadians ) / m_ego_state.wheel_base) * toDegrees;
    m_ego_state.pose.theta += dt * angular_vel_deg_per_sec;

    if (fabs(m_ego_state.pose.theta) > 360.0)
        m_ego_state.pose.theta -= m_ego_state.pose.theta/fabs(m_ego_state.pose.theta) * 360;

    m_ego_state.pose.x += dt * m_ego_state.vel * cos( m_ego_state.pose.theta * toRadians );
    m_ego_state.pose.y += dt * m_ego_state.vel * sin( m_ego_state.pose.theta * toRadians );

    m_ego_state.vel += dt * m_ego_state.accel;

    simulator_msgs::VehState ros_ego_state;

    ros_ego_state.header.stamp  = ros::Time::now();

    ros_ego_state.pose.x        = m_ego_state.pose.x;
    ros_ego_state.pose.y        = m_ego_state.pose.y;
    ros_ego_state.pose.theta    = m_ego_state.pose.theta;

    m_ego_state_pub.publish( ros_ego_state );
}

}
