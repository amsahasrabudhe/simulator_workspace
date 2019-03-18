#include "parallel_cost_calculations.cuh"

namespace cuda_mp
{

__global__
void calculate_cost(mp::Node* device_node_array)
{
    printf("\n Steering angle: %f ", device_node_array[threadIdx.x].steering);
}

void calculateCost(mp::Node* node, const mp::PlannerConfig& config)
{
    std::vector<mp::Node> child_nodes = mp::getChildNodes(node, config);

    //convert to vector to array
    mp::Node* host_node_array = child_nodes.data();

    uint nodeSize = sizeof (mp::Node);
    uint totalNodesSize = child_nodes.size() * nodeSize;

    mp::Node* device_node_array;

    cudaMalloc ((void**)&device_node_array, totalNodesSize);
    if (cudaGetLastError () != cudaSuccess)
        std::cout<<"Node array cudaMalloc booommm!!!!"<<std::endl;

    cudaMemcpy (device_node_array, host_node_array, totalNodesSize, cudaMemcpyHostToDevice);
    if (cudaGetLastError () != cudaSuccess)
        std::cout<<"Node array memCopy booommm!!!!"<<std::endl;

    calculate_cost <<<1,child_nodes.size()>>> (device_node_array);
}

}
