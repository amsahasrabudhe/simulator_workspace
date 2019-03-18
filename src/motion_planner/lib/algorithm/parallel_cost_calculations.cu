#include "parallel_cost_calculations.cuh"

namespace cuda_mp
{

__global__
void calculate_cost()
{
    printf("\n Thread id printing from GPU: %d ", threadIdx.x);
}

void calculateCost(mp::Node* node, const mp::PlannerConfig& config)
{
    std::vector<mp::Node> child_nodes = mp::getChildNodes(node, config);

    calculate_cost <<<2,2>>> ();
}

}
