#include <stdio.h>

#include <cuda.h>
#include <cuda_runtime.h>
#include <curand_kernel.h>

#include "astar/Node.hpp"

namespace cuda_mp
{
    void calculateCost(const mp::Node* node);
}
