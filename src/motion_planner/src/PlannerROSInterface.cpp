﻿/// @file

#include <math.h>
#include <XmlRpc.h>

#include <lib/helper_functions/boost_geometry_helper_functions.hpp>

#include "motion_planner/PlannerROSInterface.hpp"

namespace mp
{

PlannerROSInterface::PlannerROSInterface(const ros::NodeHandle& nh):
    m_nh(nh),
    m_last_update_time(ros::Time::now())
{}

void PlannerROSInterface::initialize()
{
    m_overall_info = std::make_shared<OverallInfo>();

    loadConfigFromParameterServer();
    loadSceneDetailsFromParameterServer();

    setupEgoVehicle();

    m_parallel_mp_algo = std::make_shared<NonholonomicAStar>(m_nh, m_overall_info, m_config);
    m_parallel_mp_algo->initialize();

    m_visualizer = std::make_shared<PlannerVisualizer>(m_nh, m_overall_info, m_config);
    m_visualizer->initialize();

    m_ego_state_pub = m_nh.advertise<simulator_msgs::EgoVehicle>(m_config.ego_veh_state_out_topic, 1, false);

    m_traffic_states_sub = m_nh.subscribe<simulator_msgs::TrafficVehicles>(m_config.traffic_veh_states_in_topic, 1, &PlannerROSInterface::trafficStatesReceived, this);

    m_update_timer = m_nh.createTimer(ros::Duration(m_config.update_time_s), &PlannerROSInterface::update, this);
    m_update_timer.start();
}

void PlannerROSInterface::update(const ros::TimerEvent& /*event*/)
{
    /// Update parallel motion planner algorithm
    m_parallel_mp_algo->planPath();
    m_visualizer->update();
    updateEgoVehicleState();

    broadcastTransforms();
    publishEgoVehicleState();
}

void PlannerROSInterface::loadConfigFromParameterServer()
{ 
    m_config.ego_veh_state_out_topic     = m_nh.param("/motion_planner/ego_veh_state_out_topic", static_cast<std::string>("/ego_veh_state"));
    m_config.traffic_veh_states_in_topic = m_nh.param("/motion_planner/traffic_veh_states_in_topic", static_cast<std::string>("/traffic_veh_states"));
    
    m_config.max_vel_mps      = m_nh.param("/motion_planner/max_vel_mps", 0.0);
    m_config.max_accel_mpss   = m_nh.param("/motion_planner/max_accel_mpss", 0.0);
    m_config.max_jerk_mpsss   = m_nh.param("/motion_planner/max_jerk_mpsss", 0.0);
    
    m_config.max_steering_rad = m_nh.param("/motion_planner/max_steering_rad", 0.0);
    
    m_config.update_time_s    = m_nh.param("/motion_planner/update_time_s", 0.1);
}

void PlannerROSInterface::loadSceneDetailsFromParameterServer()
{
    double lane_width = m_nh.param("/sim_scene_data/lane_width", 3.6);

    XmlRpc::XmlRpcValue xml_lanes;
    m_nh.param("/sim_scene_data/lanes", xml_lanes, xml_lanes);

    for (std::int8_t i = 0; i < xml_lanes.size(); ++i)
    {
        LaneInfo lane;
        lane.lane_width = lane_width;
        lane.lane_id = xml_lanes[i]["lane_id"];

        std::int32_t num_lane_points = xml_lanes[i]["lane_points"].size();

        for (std::int32_t j = 0; j < num_lane_points; ++j)
        {
            XmlRpc::XmlRpcValue lane_point_xml = xml_lanes[i]["lane_points"][j];

            Pose2D lane_point(lane_point_xml["x"], lane_point_xml["y"], lane_point_xml["heading_rad"]);
            lane.lane_points.push_back(lane_point);
        }

        m_overall_info->road_info.lanes.push_back(lane);
    }

    // Set road polygon
    m_overall_info->road_info.num_lanes = m_overall_info->road_info.lanes.size();
    m_overall_info->road_info.road_polygon_points = geometry::getRoadPolygonPoints( m_overall_info->road_info );
    m_overall_info->road_info.road_polygon = geometry::getRoadPolygon( m_overall_info->road_info );
}

void PlannerROSInterface::setupEgoVehicle()
{
    /// Load initial values from scene information on parameter server
    double init_x = m_nh.param("/sim_scene_data/ego_veh_pose/x", 0.0);
    double init_y = m_nh.param("/sim_scene_data/ego_veh_pose/y", 0.0);
    double init_heading = m_nh.param("/sim_scene_data/ego_veh_pose/heading_rad", 0.0);

    m_overall_info->ego_state->setPose(init_x, init_y, init_heading);
    
    // TODO: This init vel should be read from sim scene data
    m_overall_info->ego_state->setVel(3.5);
    m_overall_info->ego_state->setAccel(0);
    m_overall_info->ego_state->setSteeringAngle(0);

    m_overall_info->ego_state->polygon_points =  geometry::getVehiclePolygonPoints(*m_overall_info->ego_state);
}

void PlannerROSInterface::updateEgoVehicleState()
{
    ros::Time now = ros::Time::now();

    /// Calculate dt since last update in seconds
    double dt = now.toSec() - m_last_update_time.toSec();
    if (dt > 0.1)
    {
        dt = 0.1;
    }

    m_last_update_time = now;

    double curr_theta = m_overall_info->ego_state->pose.heading_rad;
    double curr_steering = m_overall_info->mp_info.curr_best_node.steering_rad;
    double curr_vel = m_overall_info->ego_state->vel_mps;

    m_overall_info->ego_state->pose.x_m += dt * curr_vel * cos(curr_theta);
    m_overall_info->ego_state->pose.y_m += dt * curr_vel * sin(curr_theta);
    
    double beta = dt * curr_vel * tan(curr_steering) / m_config.wheel_base;
    m_overall_info->ego_state->pose.heading_rad = std::fmod((curr_theta+beta), 2*M_PI);

    m_overall_info->ego_state->vel_mps += dt * m_overall_info->mp_info.curr_best_node.accel_mpss;

    // Set velocity to zero if braking activated
    m_overall_info->ego_state->vel_mps = std::max(0.0, m_overall_info->ego_state->vel_mps);

    m_overall_info->ego_state->polygon_points = geometry::getVehiclePolygonPoints(*m_overall_info->ego_state);
}

void PlannerROSInterface::broadcastTransforms()
{
    geometry_msgs::TransformStamped world_origin_to_vehicle_origin;

    world_origin_to_vehicle_origin.header.stamp = ros::Time::now();
    world_origin_to_vehicle_origin.header.frame_id = "world_origin";

    world_origin_to_vehicle_origin.child_frame_id = "vehicle_origin";

    world_origin_to_vehicle_origin.transform.translation.x = m_overall_info->ego_state->pose.x_m;
    world_origin_to_vehicle_origin.transform.translation.y = m_overall_info->ego_state->pose.y_m;
    world_origin_to_vehicle_origin.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, m_overall_info->ego_state->pose.heading_rad);
    world_origin_to_vehicle_origin.transform.rotation.x = q.x();
    world_origin_to_vehicle_origin.transform.rotation.y = q.y();
    world_origin_to_vehicle_origin.transform.rotation.z = q.z();
    world_origin_to_vehicle_origin.transform.rotation.w = q.w();

    m_tf_broadcaster.sendTransform(world_origin_to_vehicle_origin);
}

void PlannerROSInterface::publishEgoVehicleState()
{
    simulator_msgs::EgoVehicle ros_ego_state;

    ros_ego_state.header.stamp  = ros::Time::now();

    ros_ego_state.vehicle.pose.x        = m_overall_info->ego_state->pose.x_m;
    ros_ego_state.vehicle.pose.y        = m_overall_info->ego_state->pose.y_m;
    ros_ego_state.vehicle.pose.theta    = m_overall_info->ego_state->pose.heading_rad;

    ros_ego_state.vehicle.vel           = m_overall_info->ego_state->vel_mps;
    ros_ego_state.vehicle.accel         = m_overall_info->ego_state->accel_mpss;
    ros_ego_state.vehicle.steering      = m_overall_info->ego_state->steering_rad;

    ros_ego_state.vehicle.length        = m_overall_info->ego_state->length;
    ros_ego_state.vehicle.width         = m_overall_info->ego_state->width;

    m_ego_state_pub.publish( ros_ego_state );
}

void PlannerROSInterface::trafficStatesReceived(const simulator_msgs::TrafficVehicles::ConstPtr& data)
{
    /// Clear current list of traffic vehicles and refill with new incoming list
    m_overall_info->traffic.clear();

    for(const auto& vehicle : data->traffic)
    {
        Vehicle traffic_veh;
        
        traffic_veh.id          = static_cast<std::uint8_t>(vehicle.id);
        traffic_veh.length      = vehicle.length;
        traffic_veh.width       = vehicle.width;
        traffic_veh.pose        = Pose2D(vehicle.pose.x, vehicle.pose.y, vehicle.pose.theta);
        traffic_veh.steering_rad = vehicle.steering;
        traffic_veh.vel_mps     = vehicle.vel;
        traffic_veh.accel_mpss  = vehicle.accel;

        traffic_veh.polygon_points = geometry::getVehiclePolygonPoints(traffic_veh);

        m_overall_info->traffic.push_back(traffic_veh);
    }

}

}
