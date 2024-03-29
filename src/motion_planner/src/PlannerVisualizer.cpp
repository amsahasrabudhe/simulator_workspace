﻿/// @file

#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/Point.h"
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
    m_path_pub = m_nh.advertise<nav_msgs::Path>(m_cfg.planned_path_topic, 1, false);
}

void PlannerVisualizer::update()
{
    addVisualization();
    m_visualization_pub.publish(m_vis_msg);

    addPlannedPathSplineVis();
    m_path_pub.publish(m_planned_path_msg);
}

void PlannerVisualizer::addVisualization()
{
    // Clear previous messages
    m_vis_msg->markers.clear();

    addLanesVis();
    addEgoVehicleVis();
    addTrafficVis();
    addRoadPolygonVis();
    addEgoPolygonVis();

    addPlannedPathVis();
    addCurrChildNodesVis();
}

void PlannerVisualizer::addLanesVis()
{
    // Add markers for lanes visualization
    for (std::size_t i = 0; i < m_overall_info->road_info.lanes.size(); ++i)
    {
        const LaneInfo& lane = m_overall_info->road_info.lanes[i];
        
        visualization_msgs::Marker lane_point_marker;

        lane_point_marker.header.stamp = ros::Time::now();
        lane_point_marker.header.frame_id = "world_origin";

        lane_point_marker.id     = lane.lane_id;
        lane_point_marker.type   = visualization_msgs::Marker::LINE_STRIP;
        lane_point_marker.action = visualization_msgs::Marker::ADD;
        lane_point_marker.ns     = "Lanes";

        for (const auto& lane_pt : lane.lane_points)
        {
            geometry_msgs::Point pt;
            pt.x = lane_pt.x_m;
            pt.y = lane_pt.y_m;
            pt.z = 0.0;

            lane_point_marker.points.push_back(std::move(pt));
        }

        lane_point_marker.scale.x = lane.lane_width - 0.2;
        lane_point_marker.scale.y = lane.lane_width - 0.2;  // Subtracting width to show gap between lanes
        lane_point_marker.scale.z = 0.3;

        lane_point_marker.color.r = 0.2f;
        lane_point_marker.color.g = 0.2f;
        lane_point_marker.color.b = 0.2f;
        lane_point_marker.color.a = 1.0;

        lane_point_marker.lifetime = ros::Duration(0);
        lane_point_marker.frame_locked = true;

        m_vis_msg->markers.push_back(lane_point_marker);
    }
}

void PlannerVisualizer::addEgoVehicleVis()
{
    // Draw ego vehicle

    visualization_msgs::Marker vehicle;

    vehicle.header.stamp = ros::Time::now();
    vehicle.header.frame_id = "world_origin";

    vehicle.id     = 1000;
    vehicle.type   = visualization_msgs::Marker::CUBE;
    vehicle.action = visualization_msgs::Marker::ADD;
    vehicle.ns     = "ego_vehicle";

    vehicle.pose.position.x = m_overall_info->ego_state->pose.x_m + cos(m_overall_info->ego_state->pose.heading_rad) * m_cfg.wheel_base/2;
    vehicle.pose.position.y = m_overall_info->ego_state->pose.y_m + sin(m_overall_info->ego_state->pose.heading_rad) * m_cfg.wheel_base/2;
    vehicle.pose.position.z = m_cfg.height / 2;

    tf2::Quaternion q;
    q.setRPY(0, 0, m_overall_info->ego_state->pose.heading_rad);

    vehicle.pose.orientation.x = q.x();
    vehicle.pose.orientation.y = q.y();
    vehicle.pose.orientation.z = q.z();
    vehicle.pose.orientation.w = q.w();

    vehicle.scale.x = m_overall_info->ego_state->length;
    vehicle.scale.y = m_overall_info->ego_state->width;
    vehicle.scale.z = m_cfg.height;

    vehicle.color.r = 1.0;
    vehicle.color.g = 0.0;
    vehicle.color.b = 0.0;
    vehicle.color.a = 1.0;

    vehicle.lifetime = ros::Duration(0.1);
    vehicle.frame_locked = true;

    m_vis_msg->markers.push_back(vehicle);
}

void PlannerVisualizer::addTrafficVis()
{
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

        traffic_car.pose.position.x = m_overall_info->traffic[i].pose.x_m + cos(m_overall_info->traffic[i].pose.heading_rad) * m_cfg.wheel_base/2;
        traffic_car.pose.position.y = m_overall_info->traffic[i].pose.y_m + sin(m_overall_info->traffic[i].pose.heading_rad) * m_cfg.wheel_base/2;
        traffic_car.pose.position.z = m_cfg.height / 2;

        tf2::Quaternion q;
        q.setRPY(0, 0, m_overall_info->traffic[i].pose.heading_rad);

        traffic_car.pose.orientation.x = q.x();
        traffic_car.pose.orientation.y = q.y();
        traffic_car.pose.orientation.z = q.z();
        traffic_car.pose.orientation.w = q.w();

        traffic_car.scale.x = m_overall_info->traffic[i].length;
        traffic_car.scale.y = m_overall_info->traffic[i].width;
        traffic_car.scale.z = m_cfg.height;

        traffic_car.color.r = 0.2f;
        traffic_car.color.g = 0.7f;
        traffic_car.color.b = 0.8f;
        traffic_car.color.a = 1.0;

        traffic_car.lifetime = ros::Duration(0.1);
        traffic_car.frame_locked = true;

        m_vis_msg->markers.push_back(traffic_car);
    }
}

void PlannerVisualizer::addRoadPolygonVis()
{
    visualization_msgs::Marker boundary;

    boundary.header.stamp = ros::Time::now();
    boundary.header.frame_id = "world_origin";

    boundary.id     = 1111;
    boundary.type   = visualization_msgs::Marker::LINE_STRIP;
    boundary.action = visualization_msgs::Marker::ADD;
    boundary.ns     = "road_polygon";

    boundary.pose.position.z = 0.25;

    boundary.scale.x = 0.1;

    BoostPointList boundary_points = m_overall_info->road_info.road_polygon_points;
    for (std::size_t i = 0; i < boundary_points.size(); ++i)
    {
        geometry_msgs::Point point;
        point.x = boundary_points[i].x();
        point.y = boundary_points[i].y();
        point.z = 0;

        boundary.points.push_back(point);

        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;

        boundary.colors.push_back(color);
    }

    boundary.lifetime = ros::Duration(0.1);
    boundary.frame_locked = true;

    m_vis_msg->markers.push_back(boundary);
}

void PlannerVisualizer::addEgoPolygonVis()
{
    visualization_msgs::Marker ego_poly_marker;

    ego_poly_marker.header.stamp = ros::Time::now();
    ego_poly_marker.header.frame_id = "world_origin";

    ego_poly_marker.id     = 1112;
    ego_poly_marker.type   = visualization_msgs::Marker::LINE_STRIP;
    ego_poly_marker.action = visualization_msgs::Marker::ADD;
    ego_poly_marker.ns     = "ego_polygon";

    ego_poly_marker.pose.position.z = 0.25;

    ego_poly_marker.scale.x = 0.3;

    ego_poly_marker.color.r = 1.0;
    ego_poly_marker.color.g = 1.0;
    ego_poly_marker.color.b = 1.0;
    ego_poly_marker.color.a = 1.0;

    BoostPointList boundary_points = m_overall_info->ego_state->polygon_points;
    for (std::size_t i = 0; i < boundary_points.size(); ++i)
    {
        geometry_msgs::Point point;
        point.x = boundary_points[i].x();
        point.y = boundary_points[i].y();
        point.z = 0;

        ego_poly_marker.points.push_back(point);
    }

    ego_poly_marker.lifetime = ros::Duration(0.2);
    ego_poly_marker.frame_locked = true;

    m_vis_msg->markers.push_back(ego_poly_marker);
}

void PlannerVisualizer::addCurrChildNodesVis()
{
    // Draw child nodes evaulated in the recent update cycle
    for (std::size_t i = 0; i < m_overall_info->mp_info.curr_eval_nodes.size() ; ++i)
    {
        visualization_msgs::Marker child_node;

        child_node.header.stamp = ros::Time::now();
        child_node.header.frame_id = "world_origin";

        child_node.id     = 2000 + static_cast<int>(i);
        child_node.type   = visualization_msgs::Marker::CUBE;
        child_node.action = visualization_msgs::Marker::ADD;
        child_node.ns     = "curr_child_nodes";

        child_node.pose.position.x = m_overall_info->mp_info.curr_eval_nodes[i].pose.x_m + cos(m_overall_info->mp_info.curr_eval_nodes[i].pose.heading_rad) * m_cfg.wheel_base/2;
        child_node.pose.position.y = m_overall_info->mp_info.curr_eval_nodes[i].pose.y_m + sin(m_overall_info->mp_info.curr_eval_nodes[i].pose.heading_rad) * m_cfg.wheel_base/2;
        child_node.pose.position.z = m_cfg.height / 2;

        tf2::Quaternion q;
        q.setRPY(0, 0, m_overall_info->mp_info.curr_eval_nodes[i].pose.heading_rad);

        child_node.pose.orientation.x = q.x();
        child_node.pose.orientation.y = q.y();
        child_node.pose.orientation.z = q.z();
        child_node.pose.orientation.w = q.w();

        child_node.scale.x = 0.1;
        child_node.scale.y = 0.1;
        child_node.scale.z = 0.1/*m_cfg.height*/;

        child_node.color.r = 1.0;
        child_node.color.g = 0.3f * (i+1);
        child_node.color.b = 0.3f * (i-1);
        child_node.color.a = 0.7f;

        child_node.lifetime = ros::Duration(0.1);
        child_node.frame_locked = true;

        m_vis_msg->markers.push_back(child_node);
    }
}

void PlannerVisualizer::addPlannedPathVis()
{
    // Draw planned path
    for (std::size_t i = 0; i < m_overall_info->mp_info.planned_path.size() ; ++i)
    {
        visualization_msgs::Marker node;

        node.header.stamp = ros::Time::now();
        node.header.frame_id = "world_origin";

        node.id     = 10000 + static_cast<int>(i);
        node.type   = visualization_msgs::Marker::CUBE;
        node.action = visualization_msgs::Marker::ADD;
        node.ns     = "planned";

        node.pose.position.x = m_overall_info->mp_info.planned_path[i].pose.x_m + cos(m_overall_info->mp_info.planned_path[i].pose.heading_rad) * m_cfg.wheel_base/2;
        node.pose.position.y = m_overall_info->mp_info.planned_path[i].pose.y_m + sin(m_overall_info->mp_info.planned_path[i].pose.heading_rad) * m_cfg.wheel_base/2;
        node.pose.position.z = m_cfg.height/2 - 0.1;

        tf2::Quaternion q;
        q.setRPY(0, 0, m_overall_info->mp_info.planned_path[i].pose.heading_rad);

        node.pose.orientation.x = q.x();
        node.pose.orientation.y = q.y();
        node.pose.orientation.z = q.z();
        node.pose.orientation.w = q.w();

        node.scale.x = m_overall_info->ego_state->length;
        node.scale.y = m_overall_info->ego_state->width;
        node.scale.z = 0.1;

        node.color.r = 1.0;
        node.color.g = 0.3f * (i-1);
        node.color.b = 0.3f * (i+1);
        node.color.a = 0.7f;

        node.lifetime = ros::Duration(0.1);
        node.frame_locked = true;

        m_vis_msg->markers.push_back(node);
    }
}

void PlannerVisualizer::addPlannedPathSplineVis()
{
    m_planned_path_msg = boost::make_shared<nav_msgs::Path>();

    m_planned_path_msg->header.frame_id = "world_origin";
    m_planned_path_msg->header.stamp = ros::Time::now ();

    // Add current state
    geometry_msgs::PoseStamped curr_state;

    curr_state.header.stamp = ros::Time::now();
    curr_state.header.frame_id = "world_origin";

    curr_state.pose.position.x = m_overall_info->ego_state->pose.x_m + cos(m_overall_info->ego_state->pose.heading_rad) * m_cfg.wheel_base/2;
    curr_state.pose.position.y = m_overall_info->ego_state->pose.y_m + sin(m_overall_info->ego_state->pose.heading_rad) * m_cfg.wheel_base/2;
    curr_state.pose.position.z = m_cfg.height / 2;

    tf2::Quaternion q;
    q.setRPY(0, 0, m_overall_info->ego_state->pose.heading_rad);

    curr_state.pose.orientation.x = q.x();
    curr_state.pose.orientation.y = q.y();
    curr_state.pose.orientation.z = q.z();
    curr_state.pose.orientation.w = q.w();

    m_planned_path_msg->poses.push_back(curr_state);

    // Add planned path
    for (std::size_t i = 0; i < m_overall_info->mp_info.planned_path.size() ; ++i)
    {
        geometry_msgs::PoseStamped node;

        node.header.stamp = ros::Time::now() + ros::Duration( (i+1)*m_cfg.child_node_dt );
        node.header.frame_id = "world_origin";

        node.pose.position.x = m_overall_info->mp_info.planned_path[i].pose.x_m + cos(m_overall_info->mp_info.planned_path[i].pose.heading_rad) * m_cfg.wheel_base/2;
        node.pose.position.y = m_overall_info->mp_info.planned_path[i].pose.y_m + sin(m_overall_info->mp_info.planned_path[i].pose.heading_rad) * m_cfg.wheel_base/2;
        node.pose.position.z = m_cfg.height / 2;

        tf2::Quaternion q;
        q.setRPY(0, 0, m_overall_info->mp_info.planned_path[i].pose.heading_rad);

        node.pose.orientation.x = q.x();
        node.pose.orientation.y = q.y();
        node.pose.orientation.z = q.z();
        node.pose.orientation.w = q.w();

        m_planned_path_msg->poses.push_back(node);
    }
}

}
