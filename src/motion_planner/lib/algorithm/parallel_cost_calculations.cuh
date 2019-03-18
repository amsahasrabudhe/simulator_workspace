#include <stdio.h>

#include <cuda.h>
#include <cuda_runtime.h>
#include <curand_kernel.h>

#include "astar/Node.hpp"
#include "lib/helper_functions/motion_planning_helper_functions.hpp"

namespace cuda_mp
{
    void calculateCost(mp::Node* node, const mp::PlannerConfig& config);
}
