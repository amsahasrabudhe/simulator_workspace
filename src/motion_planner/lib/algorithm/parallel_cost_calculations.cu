#include "parallel_cost_calculations.cuh"

namespace cuda_mp
{

__global__
void calculate_cost()
{
    printf("\n Thread id printing from GPU: %d ", threadIdx.x);
}

void calculateCost(const mp::Node* node)
{
    calculate_cost <<<2,2>>> ();
}

}
