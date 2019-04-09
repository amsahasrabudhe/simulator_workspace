#include "parallel_cost_calculations.cuh"

namespace cuda_mp
{

/**********************************DEVICE FUNCTIONS***************************************/

__device__
double Angle2D(double x1, double y1, double x2, double y2)
{
   double theta_1, theta_2;
   double dtheta;

   theta_1 = atan2(y1,x1);
   theta_2 = atan2(y2,x2);

   dtheta = theta_2 - theta_1;

   while (dtheta > M_PI)
      dtheta -= 2*M_PI;

   while (dtheta < -M_PI)
      dtheta += 2*M_PI;

   return(dtheta);
}

__device__
bool isPointInsidePolygon(const mp::Vec2D& point, const mp::Vec2D* polygon, const uint polygon_point_count)
{
    double angle = 0.0;

    mp::Vec2D p1, p2;

    for (uint i = 0; i < polygon_point_count; ++i)
    {
        p1.x = polygon[i].x - point.x;
        p1.y = polygon[i].y - point.y;

        p2.x = polygon[(i+1)%polygon_point_count].x - point.x;
        p2.y = polygon[(i+1)%polygon_point_count].y - point.y;

        angle += Angle2D (p1.x, p1.y, p2.x, p2.y);
    }

    if (fabs(angle) < M_PI)
        return false;

    return true;
}

__device__
bool isNodeInsideRoad(const mp::Vec2D* child_node_polygon_points, const mp::Vec2D* road_polygon, const uint road_polygon_point_count)
{
    bool inside = true;

    for (uint i=0; i < 4; ++i)
    {
        inside = isPointInsidePolygon (child_node_polygon_points[i], road_polygon, road_polygon_point_count);

        // Exit loop if even one point is outside the road polygon
        if (!inside)
        {
            break;
        }
    }

    return inside;
}

__device__
bool nodeCollidesWithTraffic(const mp::Vec2D* child_node_polygon_points, const mp::Vec2D* traffic_polygons, const uint traffic_veh_count)
{
    bool is_colliding = false;

    for (uint veh_num = 0; veh_num < traffic_veh_count; ++veh_num)
    {
        for (uint i = 0; i < 4; ++i)
        {
            is_colliding = isPointInsidePolygon(child_node_polygon_points[i],
                                                (traffic_polygons + 4*veh_num), 4);

            if (is_colliding)
                return true;
        }
    }

    return false;
}

__device__
void calculateVehiclePolygon(const mp::Node& node, mp::Vec2D* veh_polygon_points)
{
    double yaw = node.pose.theta;

    double front      = 3.869;    // approx (vehicle.length/2 + vehicle.wheel_base/2)
    double rear       = 0.910;    // approx (vehicle.length/2 - vehicle.wheel_base/2)
    double width_by_2 = 1.05;

    mp::Vec2D front_left;
    front_left.x = cos(yaw)*front + cos(yaw + M_PI_2)*width_by_2 + node.pose.x;
    front_left.y = sin(yaw)*front + sin(yaw + M_PI_2)*width_by_2 + node.pose.y;

    mp::Vec2D front_right;
    front_right.x = cos(yaw)*front + cos(yaw - M_PI_2)*width_by_2 + node.pose.x;
    front_right.y = sin(yaw)*front + sin(yaw - M_PI_2)*width_by_2 + node.pose.y;

    mp::Vec2D rear_left;
    rear_left.x = - cos(yaw)*rear + cos(yaw + M_PI_2)*width_by_2 + node.pose.x;
    rear_left.y = - sin(yaw)*rear + sin(yaw + M_PI_2)*width_by_2 + node.pose.y;

    mp::Vec2D rear_right;
    rear_right.x = - cos(yaw)*rear + cos(yaw - M_PI_2)*width_by_2 + node.pose.x;
    rear_right.y = - sin(yaw)*rear + sin(yaw - M_PI_2)*width_by_2 + node.pose.y;

    veh_polygon_points[0] = rear_left;
    veh_polygon_points[1] = front_left;
    veh_polygon_points[2] = front_right;
    veh_polygon_points[3] = rear_right;
}



__global__
void calculate_cost(mp::Node* device_child_node_array,
                    mp::Vec2D* road_polygon, const uint road_polygon_point_count,
                    mp::Vec2D* traffic_polygons, const uint traffic_veh_count)
{
//    printf("\n Theta: %f, Pose: %f , %f ", device_node_array[threadIdx.x].pose.theta, device_node_array[threadIdx.x].pose.x, device_node_array[threadIdx.x].pose.y);

    mp::Vec2D child_node_polygon_points[4];
    calculateVehiclePolygon( device_child_node_array[threadIdx.x], child_node_polygon_points );

    bool inside_road_boundary = isNodeInsideRoad(child_node_polygon_points, road_polygon, road_polygon_point_count);

    bool is_colliding = nodeCollidesWithTraffic (child_node_polygon_points, traffic_polygons, traffic_veh_count);

}

/***********************************HOST FUNCTIONS****************************************/

void checkCudaError(const std::string& error_msg)
{
    if (cudaGetLastError () != cudaSuccess)
        std::cout<<error_msg<<std::endl;
}

void calculateCost(std::vector<mp::Node>& child_nodes, const mp::PlannerConfig& config, const std::shared_ptr<mp::OverallInfo>& overall_info)
{
    //convert to child node vector to array
    mp::Node* host_child_node_array = child_nodes.data();

    uint totalChildNodes = child_nodes.size();
    uint totalChildNodesSize = totalChildNodes * sizeof(mp::Node);

    mp::Node* device_child_node_array;

    cudaMalloc ((void**)&device_child_node_array, totalChildNodesSize);
    checkCudaError ( "Node array cudaMalloc booommm!!!!" );

    cudaMemcpy (device_child_node_array, host_child_node_array, totalChildNodesSize, cudaMemcpyHostToDevice);
    checkCudaError ( "Node array memCopy booommm!!!!" );

    // Convert road polygon to array
    std::vector<mp::Vec2D> road_polygon = getRoadPolygon (overall_info->road_info);
    mp::Vec2D* host_road_polygon = road_polygon.data();

    uint totalRoadPolygonPoints = road_polygon.size();
    uint totalRoadPolygonSize = totalRoadPolygonPoints * sizeof(mp::Vec2D);

    mp::Vec2D* device_road_polygon;

    cudaMalloc ((void**)&device_road_polygon, totalRoadPolygonSize);
    checkCudaError ( "Road polygon cudaMalloc booommm!!!!" );

    cudaMemcpy (device_road_polygon, host_road_polygon, totalRoadPolygonSize, cudaMemcpyHostToDevice);
    checkCudaError ( "Road polygon memCopy booommm!!!!" );

    // Convert traffic polygons to array
    std::vector<mp::Vec2D> traffic_polygons = getTrafficPolygons (overall_info->traffic);

    mp::Vec2D* host_traffic_polygons_array = traffic_polygons.data();

    uint total_traffic_vehicles = overall_info->traffic.size();
    uint total_traffic_polygons_size = total_traffic_vehicles * 4 * sizeof (mp::Vec2D);

    mp::Vec2D* device_traffic_polygons_array;

    cudaMalloc ((void**)&device_traffic_polygons_array, total_traffic_polygons_size);
    checkCudaError ( "Traffic polygons array cudaMalloc booommm!!!!" );

    cudaMemcpy (device_traffic_polygons_array, host_traffic_polygons_array, total_traffic_polygons_size, cudaMemcpyHostToDevice);

    ros::Time start_time = ros::Time::now();

    // Kernel call
    calculate_cost <<<1,child_nodes.size()>>> (device_child_node_array,
                                               device_road_polygon, totalRoadPolygonPoints,
                                               device_traffic_polygons_array, total_traffic_vehicles);
    cudaDeviceSynchronize();

    // Copy all necessary data from GPU to CPU
    cudaMemcpy (host_child_node_array, device_child_node_array, totalChildNodesSize, cudaMemcpyDeviceToHost);

    ros::Time end_time = ros::Time::now();
    std::cout<<"\nCUDA Execution time : "<<(end_time-start_time).toSec()<<std::endl;

    // Free memory on the gpu
    cudaFree (device_child_node_array);
    cudaFree (device_road_polygon);
    cudaFree (device_traffic_polygons_array);

    // Store nodes evaluated in the recent cycle
    overall_info->mp_info.curr_eval_nodes.clear();
    for (uint i = 0; i < totalChildNodes; ++i)
    {
        overall_info->mp_info.curr_eval_nodes.push_back (host_child_node_array[i]);
    }
}

std::vector<mp::Vec2D> getRoadPolygon(const mp::RoadInfo& road_info)
{
    std::vector<mp::Vec2D> polygon_points;

    mp::LaneInfo leftmost_lane = road_info.lanes[0];
    double leftmost_lane_half_width = leftmost_lane.lane_width / 2;

    for ( std::vector<mp::Pose2D>::const_iterator iter = leftmost_lane.lane_points.begin();
          iter != leftmost_lane.lane_points.end(); ++iter )
    {
        mp::Vec2D point;
        point.x = leftmost_lane_half_width * cos(iter->theta + M_PI_2) + iter->x;
        point.y = leftmost_lane_half_width * sin(iter->theta + M_PI_2) + iter->y;

        polygon_points.push_back( point );
    }

    // Get rightmost lane edge
    mp::LaneInfo rightmost_lane = road_info.lanes[ road_info.num_lanes - 1 ];
    double rightmost_lane_half_width = rightmost_lane.lane_width / 2;

    for ( std::vector<mp::Pose2D>::const_reverse_iterator iter = rightmost_lane.lane_points.rbegin();
          iter != rightmost_lane.lane_points.rend(); ++iter )
    {
        mp::Vec2D point;
        point.x = rightmost_lane_half_width * cos(iter->theta - M_PI_2) + iter->x;
        point.y = rightmost_lane_half_width * sin(iter->theta - M_PI_2) + iter->y;

        polygon_points.push_back( point );
    }

    return polygon_points;
}

std::vector<mp::Vec2D> getTrafficPolygons(const std::vector<mp::Vehicle>& traffic)
{
    std::vector<mp::Vec2D> traffic_polygons;

    for (const auto& vehicle : traffic)
    {
        double yaw = vehicle.pose.theta;

//        double front      = 3.869;    // approx (vehicle.length/2 + vehicle.wheel_base/2)
//        double rear       = 0.910;    // approx (vehicle.length/2 - vehicle.wheel_base/2)
        double front      = 2.389;    // vehicle.length/2 (since simulator publishes midpoint as pose)
        double rear       = 2.389;    // vehicle.length/2
        double width_by_2 = 1.05;

        mp::Vec2D front_left;
        front_left.x = cos(yaw)*front + cos(yaw + M_PI_2)*width_by_2 + vehicle.pose.x;
        front_left.y = sin(yaw)*front + sin(yaw + M_PI_2)*width_by_2 + vehicle.pose.y;

        mp::Vec2D front_right;
        front_right.x = cos(yaw)*front + cos(yaw - M_PI_2)*width_by_2 + vehicle.pose.x;
        front_right.y = sin(yaw)*front + sin(yaw - M_PI_2)*width_by_2 + vehicle.pose.y;

        mp::Vec2D rear_left;
        rear_left.x = - cos(yaw)*rear + cos(yaw + M_PI_2)*width_by_2 + vehicle.pose.x;
        rear_left.y = - sin(yaw)*rear + sin(yaw + M_PI_2)*width_by_2 + vehicle.pose.y;

        mp::Vec2D rear_right;
        rear_right.x = - cos(yaw)*rear + cos(yaw - M_PI_2)*width_by_2 + vehicle.pose.x;
        rear_right.y = - sin(yaw)*rear + sin(yaw - M_PI_2)*width_by_2 + vehicle.pose.y;

        traffic_polygons.push_back (rear_left);
        traffic_polygons.push_back (front_left);
        traffic_polygons.push_back (front_right);
        traffic_polygons.push_back (rear_right);
    }

    return traffic_polygons;
}


}
