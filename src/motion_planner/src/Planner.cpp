/// @file

#include "motion_planner/Planner.hpp"

#include <math.h>
#include <simulator_msgs/EgoVehicle.h>

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

    m_ego_state_pub = m_nh.advertise<simulator_msgs::EgoVehicle>(m_config.ego_veh_state_out_topic, 1, false);

    m_update_timer = m_nh.createTimer(ros::Duration(m_config.update_time_s), &Planner::update, this);
    m_update_timer.start();
}

void Planner::setupEgoVehicle()
{
    m_ego_state.setPose(6.326489009482704, 24.673307136982547, 15*toRadians);
    m_ego_state.setVel(0);
    m_ego_state.setAccel(1);
    m_ego_state.setSteeringAngle(-1);
}

void Planner::update(const ros::TimerEvent& event)
{
    updateEgoVehicleState();

    publishEgoVehicleState();
}

void Planner::updateEgoVehicleState()
{
    ros::Time now = ros::Time::now();
    /// Calculate dt since last update in seconds
    double dt = now.toSec() - m_last_update_time.toSec();

    m_last_update_time = now;

    Pose2D front_wheel_pos = m_ego_state.pose
            + Pose2D(cos(m_ego_state.pose.theta),sin(m_ego_state.pose.theta),0)*(m_ego_state.wheel_base/2);

    Pose2D rear_wheel_pos = m_ego_state.pose
            - Pose2D(cos(m_ego_state.pose.theta),sin(m_ego_state.pose.theta),0)*(m_ego_state.wheel_base/2);

    front_wheel_pos.x += dt * m_ego_state.vel * cos(m_ego_state.pose.theta + m_ego_state.steering*toRadians);
    front_wheel_pos.y += dt * m_ego_state.vel * sin(m_ego_state.pose.theta + m_ego_state.steering*toRadians);

    rear_wheel_pos.x += dt * m_ego_state.vel * cos(m_ego_state.pose.theta);
    rear_wheel_pos.y += dt * m_ego_state.vel * sin(m_ego_state.pose.theta);

    m_ego_state.pose = (front_wheel_pos+rear_wheel_pos)/2;

    m_ego_state.pose.theta = atan2( front_wheel_pos.y-rear_wheel_pos.y, front_wheel_pos.x-rear_wheel_pos.x );

    m_ego_state.vel += dt * m_ego_state.accel;
}

void Planner::publishEgoVehicleState()
{
    simulator_msgs::EgoVehicle ros_ego_state;

    ros_ego_state.header.stamp  = ros::Time::now();

    ros_ego_state.vehicle.pose.x        = m_ego_state.pose.x;
    ros_ego_state.vehicle.pose.y        = m_ego_state.pose.y;
    ros_ego_state.vehicle.pose.theta    = m_ego_state.pose.theta;

    ros_ego_state.vehicle.vel = m_ego_state.vel;
    ros_ego_state.vehicle.accel = m_ego_state.accel;
    ros_ego_state.vehicle.steering = m_ego_state.steering;

    ros_ego_state.vehicle.length = m_ego_state.length;
    ros_ego_state.vehicle.width = m_ego_state.width;

    m_ego_state_pub.publish( ros_ego_state );
}

/*///Basic implementation of bicycle model
void Planner::updateEgoVehicleState()
{
    ros::Time now = ros::Time::now();
    /// Calculate dt since last update in seconds
    double dt = now.toSec() - m_last_update_time.toSec();

    m_last_update_time = now;

    m_ego_state.pose.theta += dt * (m_ego_state.vel * tan( m_ego_state.steering*toRadians ) / m_ego_state.wheel_base);

    m_ego_state.vel += dt * m_ego_state.accel;

    m_ego_state.pose.x += dt * m_ego_state.vel * cos( m_ego_state.pose.theta );
    m_ego_state.pose.y += dt * m_ego_state.vel * sin( m_ego_state.pose.theta );
}*/

}
