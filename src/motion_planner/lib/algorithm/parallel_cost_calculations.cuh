#include <stdio.h>

#include <cuda.h>
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <ros/ros.h>
#include <ros/time.h>

#include "astar/Node.hpp"
#include "lib/helper_functions/boost_geometry_helper_functions.hpp"
#include "lib/helper_functions/motion_planning_helper_functions.hpp"
#include "lib/types/OverallInfo.hpp"
#include "lib/types/Vec2D.hpp"

namespace cuda_mp
{
    void checkCudaError(const std::string& error_msg);

    void calculateCost(std::vector<mp::Node>& child_nodes, const mp::PlannerConfig& config, const std::shared_ptr<mp::OverallInfo>& overall_info);

    std::vector<mp::Vec2D> getRoadPolygon(const mp::RoadInfo& road_info);

    std::vector<mp::Vec2D> getTrafficPolygons(const std::vector<mp::Vehicle>& traffic);
}
