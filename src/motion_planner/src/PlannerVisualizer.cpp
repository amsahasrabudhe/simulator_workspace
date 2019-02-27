﻿/// @file

#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

#include "motion_planner/PlannerVisualizer.hpp"

namespace mp
{

PlannerVisualizer::PlannerVisualizer(const ros::NodeHandle& nh, const std::shared_ptr<OverallInfo>& info, const PlannerConfig& cfg):
    m_nh(nh),
    m_cfg(cfg),
    m_overall_info(info)
{

}

void PlannerVisualizer::initialize()
{
    m_vis_msg = boost::make_shared<visualization_msgs::MarkerArray>();

    m_visualization_pub = m_nh.advertise<visualization_msgs::MarkerArray>(m_cfg.visualization_topic, 1, false);
}

void PlannerVisualizer::update()
{
    addVisualization();

    m_visualization_pub.publish(m_vis_msg);
}

void PlannerVisualizer::addVisualization()
{
    // Clear previous messages
    m_vis_msg->markers.clear();

    // Add markers for current lane points used for spline generation
    for (std::size_t i = 0; i < m_overall_info->curr_poly_lanepoints.size(); ++i)
    {
        visualization_msgs::Marker point;

        point.header.stamp = ros::Time::now();
        point.header.frame_id = "world_origin";

        point.id     = static_cast<int>(i);
        point.type   = visualization_msgs::Marker::SPHERE;
        point.action = visualization_msgs::Marker::ADD;
        point.ns     = "spline_points";

        point.pose.position.x = m_overall_info->curr_poly_lanepoints[i].x;
        point.pose.position.y = m_overall_info->curr_poly_lanepoints[i].y;
        point.pose.position.z = 0.1;

        point.scale.x = 0.1;
        point.scale.y = 0.1;
        point.scale.z = 0.1;

        point.color.r = 0.0;
        point.color.g = 1.0;
        point.color.b = 0.0;
        point.color.a = 1.0;

        point.lifetime = ros::Duration(0.1);
        point.frame_locked = true;

        m_vis_msg->markers.push_back(point);
    }

    // Add markers for lanes visualization
    for (std::size_t i = 0; i < m_overall_info->road_info.lanes.size(); ++i)
    {
        LaneInfo lane = m_overall_info->road_info.lanes[i];
        for (std::size_t j = 0; j < lane.lane_points.size(); ++j)
        {
            visualization_msgs::Marker lane_point_marker;

            lane_point_marker.header.stamp = ros::Time::now();
            lane_point_marker.header.frame_id = "world_origin";

            lane_point_marker.id     = 100*static_cast<int>(i) + static_cast<int>(j);
            lane_point_marker.type   = visualization_msgs::Marker::CUBE;
            lane_point_marker.action = visualization_msgs::Marker::ADD;
            lane_point_marker.ns     = "lanes";

            lane_point_marker.pose.position.x = lane.lane_points[j].x;
            lane_point_marker.pose.position.y = lane.lane_points[j].y;
            lane_point_marker.pose.position.z = 0;

            tf2::Quaternion q;
            q.setRPY(0, 0, lane.lane_points[j].theta);

            lane_point_marker.pose.orientation.x = q.x();
            lane_point_marker.pose.orientation.y = q.y();
            lane_point_marker.pose.orientation.z = q.z();
            lane_point_marker.pose.orientation.w = q.w();

            lane_point_marker.scale.x = 2.3;
            lane_point_marker.scale.y = m_overall_info->road_info.lanes[i].lane_width - 0.1;  // - 0.1 to show gap between lanes
            lane_point_marker.scale.z = 0.1;

            lane_point_marker.color.r = 0.3f;
            lane_point_marker.color.g = 0.3f;
            lane_point_marker.color.b = 0.3f;
            lane_point_marker.color.a = 1.0;

            lane_point_marker.lifetime = ros::Duration(0);
            lane_point_marker.frame_locked = true;

            m_vis_msg->markers.push_back(lane_point_marker);
        }
    }

    // Draw ego vehicle

    visualization_msgs::Marker vehicle;

    vehicle.header.stamp = ros::Time::now();
    vehicle.header.frame_id = "world_origin";

    vehicle.id     = 1000;
    vehicle.type   = visualization_msgs::Marker::CUBE;
    vehicle.action = visualization_msgs::Marker::ADD;
    vehicle.ns     = "ego_vehicle";

    vehicle.pose.position.x = m_overall_info->ego_state->pose.x + cos(m_overall_info->ego_state->pose.theta) * m_overall_info->ego_state->wheel_base/2;
    vehicle.pose.position.y = m_overall_info->ego_state->pose.y + sin(m_overall_info->ego_state->pose.theta) * m_overall_info->ego_state->wheel_base/2;
    vehicle.pose.position.z = m_overall_info->ego_state->height / 2;

    tf2::Quaternion q;
    q.setRPY(0, 0, m_overall_info->ego_state->pose.theta);

    vehicle.pose.orientation.x = q.x();
    vehicle.pose.orientation.y = q.y();
    vehicle.pose.orientation.z = q.z();
    vehicle.pose.orientation.w = q.w();

    vehicle.scale.x = m_overall_info->ego_state->length;
    vehicle.scale.y = m_overall_info->ego_state->width;
    vehicle.scale.z = m_overall_info->ego_state->height;

    vehicle.color.r = 1.0;
    vehicle.color.g = 0.0;
    vehicle.color.b = 0.0;
    vehicle.color.a = 1.0;

    vehicle.lifetime = ros::Duration(0.1);
    vehicle.frame_locked = true;

    m_vis_msg->markers.push_back(vehicle);

    // Draw traffic vehicles

    for (std::size_t i = 0; i < m_overall_info->traffic.size() ; ++i)
    {
        visualization_msgs::Marker traffic_car;

        traffic_car.header.stamp = ros::Time::now();
        traffic_car.header.frame_id = "world_origin";

        traffic_car.id     = 1000 + static_cast<int>(i);
        traffic_car.type   = visualization_msgs::Marker::CUBE;
        traffic_car.action = visualization_msgs::Marker::ADD;
        traffic_car.ns     = "traffic";

        traffic_car.pose.position.x = m_overall_info->traffic[i].pose.x + cos(m_overall_info->traffic[i].pose.theta) * m_overall_info->traffic[i].wheel_base/2;
        traffic_car.pose.position.y = m_overall_info->traffic[i].pose.y + sin(m_overall_info->traffic[i].pose.theta) * m_overall_info->traffic[i].wheel_base/2;
        traffic_car.pose.position.z = m_overall_info->traffic[i].height / 2;

        tf2::Quaternion q;
        q.setRPY(0, 0, m_overall_info->traffic[i].pose.theta);

        traffic_car.pose.orientation.x = q.x();
        traffic_car.pose.orientation.y = q.y();
        traffic_car.pose.orientation.z = q.z();
        traffic_car.pose.orientation.w = q.w();

        traffic_car.scale.x = m_overall_info->traffic[i].length;
        traffic_car.scale.y = m_overall_info->traffic[i].width;
        traffic_car.scale.z = m_overall_info->traffic[i].height;

        traffic_car.color.r = 0.2f;
        traffic_car.color.g = 0.7f;
        traffic_car.color.b = 0.8f;
        traffic_car.color.a = 1.0;

        traffic_car.lifetime = ros::Duration(0.1);
        traffic_car.frame_locked = true;

        m_vis_msg->markers.push_back(traffic_car);
    }
}

}
