#include "parallel_cost_calculations.cuh"

namespace cuda_mp
{

__global__
void calculate_cost(mp::Node* device_node_array)
{
    printf("\n Theta: %f, Pose: %f , %f ", device_node_array[threadIdx.x].pose.theta, device_node_array[threadIdx.x].pose.x, device_node_array[threadIdx.x].pose.y);
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
    for (int i = 0; i < totalNodes; ++i)
    {
        overall_info->mp_info.curr_eval_nodes.push_back (host_node_array[i]);
    }
}

}
