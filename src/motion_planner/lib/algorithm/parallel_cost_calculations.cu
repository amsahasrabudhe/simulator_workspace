#include "parallel_cost_calculations.cuh"

namespace cuda_mp
{

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
    veh_polygon_points[4] = rear_left;
}

__device__
bool isPointInsidePolygon(mp::Vec2D* polygon, const mp::Node& node, const mp::Vec2D& point)
{
    calculateVehiclePolygon(node, polygon);



    return true;
}

__global__
void calculate_cost(mp::Node* device_node_array)
{
    printf("\n Theta: %f, Pose: %f , %f ", device_node_array[threadIdx.x].pose.theta, device_node_array[threadIdx.x].pose.x, device_node_array[threadIdx.x].pose.y);

    mp::Vec2D polygon[5];

    mp::Vec2D test;
    test.x = 25.0;
    test.y = 25.0;

    isPointInsidePolygon (polygon, device_node_array[threadIdx.x], test);
}

void calculateCost(std::vector<mp::Node>& child_nodes, const mp::PlannerConfig& config, const std::shared_ptr<mp::OverallInfo>& overall_info)
{
    //convert to vector to array
    mp::Node* host_node_array = child_nodes.data();

    uint nodeSize = sizeof (child_nodes[0]);
    uint totalNodes = child_nodes.size();
    uint totalNodesSize = totalNodes * nodeSize;

    mp::Node* device_node_array;

    cudaMalloc ((void**)&device_node_array, totalNodesSize);
    if (cudaGetLastError () != cudaSuccess)
        std::cout<<"Node array cudaMalloc booommm!!!!"<<std::endl;

    cudaMemcpy (device_node_array, host_node_array, totalNodesSize, cudaMemcpyHostToDevice);
    if (cudaGetLastError () != cudaSuccess)
        std::cout<<"Node array memCopy booommm!!!!"<<std::endl;

    calculate_cost <<<1,child_nodes.size()>>> (device_node_array);
    cudaDeviceSynchronize();

    cudaMemcpy (host_node_array, device_node_array, totalNodesSize, cudaMemcpyDeviceToHost);

    // Free memory on the gpu
    cudaFree (device_node_array);

    // Store nodes evaluated in the recent cycle
    overall_info->mp_info.curr_eval_nodes.clear();
    for (uint i = 0; i < totalNodes; ++i)
    {
        overall_info->mp_info.curr_eval_nodes.push_back (host_node_array[i]);
    }
}

}
