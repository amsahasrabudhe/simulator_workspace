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
    m_overall_info = std::make_shared<OverallInfo>();

    setupEgoVehicle();

    m_ego_state_pub = m_nh.advertise<simulator_msgs::EgoVehicle>(m_config.ego_veh_state_out_topic, 1, false);

    m_traffic_states_sub = m_nh.subscribe<simulator_msgs::TrafficVehicles>(m_config.traffic_veh_states_in_topic, 1, &Planner::trafficStatesReceived, this);

    m_update_timer = m_nh.createTimer(ros::Duration(m_config.update_time_s), &Planner::update, this);
    m_update_timer.start();
}

void Planner::setupEgoVehicle()
{
    m_overall_info->ego_state->setPose(4.217659339655136, 25.024778748620474, 0);
    m_overall_info->ego_state->setVel(0);
    m_overall_info->ego_state->setAccel(1);
    m_overall_info->ego_state->setSteeringAngle(0);
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

    double curr_theta = m_overall_info->ego_state->pose.theta;
    double curr_vel = m_overall_info->ego_state->vel;

    m_overall_info->ego_state->pose.x += dt * curr_vel * cos(curr_theta);
    m_overall_info->ego_state->pose.y += dt * curr_vel * sin(curr_theta);

    m_overall_info->ego_state->pose.theta = curr_theta + dt * (curr_vel/m_overall_info->ego_state->wheel_base)*tan(m_overall_info->ego_state->steering);
    
    m_overall_info->ego_state->vel += dt * m_overall_info->ego_state->accel;
}

void Planner::publishEgoVehicleState()
{
    simulator_msgs::EgoVehicle ros_ego_state;

    ros_ego_state.header.stamp  = ros::Time::now();

    ros_ego_state.vehicle.pose.x        = m_overall_info->ego_state->pose.x;
    ros_ego_state.vehicle.pose.y        = m_overall_info->ego_state->pose.y;
    ros_ego_state.vehicle.pose.theta    = m_overall_info->ego_state->pose.theta;

    ros_ego_state.vehicle.vel           = m_overall_info->ego_state->vel;
    ros_ego_state.vehicle.accel         = m_overall_info->ego_state->accel;
    ros_ego_state.vehicle.steering      = m_overall_info->ego_state->steering;

    ros_ego_state.vehicle.length        = m_overall_info->ego_state->length;
    ros_ego_state.vehicle.width         = m_overall_info->ego_state->width;

    m_ego_state_pub.publish( ros_ego_state );
}

void Planner::trafficStatesReceived(const simulator_msgs::TrafficVehicles::ConstPtr& data)
{
    /// Clear current list of traffic vehicles and refill with new incoming list
    m_overall_info->traffic.clear();

    for(auto vehicle : data->traffic)
    {
        Vehicle traffic_veh = Vehicle();
        
        traffic_veh.id          = vehicle.id;
        traffic_veh.length      = vehicle.length;
        traffic_veh.width       = vehicle.width;
        traffic_veh.pose.x      = vehicle.pose.x;
        traffic_veh.pose.y      = vehicle.pose.y;
        traffic_veh.pose.theta  = vehicle.pose.theta;
        traffic_veh.steering    = vehicle.steering;
        traffic_veh.vel         = vehicle.vel;
        traffic_veh.accel       = vehicle.accel;

        m_overall_info->traffic.push_back(traffic_veh);
    }

}

}
