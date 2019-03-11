#include "parallel_cost_calculations.cuh"

namespace cuda_mp
{

__global__ void calculate_cost(void)
{
    printf("\n Thread id printing from GPU: %d", threadIdx.x);
}

void calculateCost()
{
    calculate_cost <<<2,2>>> ();
}

}
