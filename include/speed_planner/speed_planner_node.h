#ifndef SPEED_PLANNER_NODE_ROS_H
#define SPEED_PLANNER_NODE_ROS_H

#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include <random>
#include <chrono>
#include <cstring>
#include <memory>

#include "speed_planner/convex_speed_optimizer.h"
#include "speed_planner/obstacle.h"
#include "speed_planner/vehicle_info.h"
#include "speed_planner/collision_checker.h"
#include "speed_planner/trajectory.h"
#include "speed_planner/utils.h"

class SpeedPlannerNode
{
    public:
        SpeedPlannerNode();
        ~SpeedPlannerNode() = default;

    private:
       

  
        std::unique_ptr<ConvexSpeedOptimizer> speedOptimizer_;
        std::unique_ptr<VehicleInfo> ego_vehicle_ptr_;
        std::unique_ptr<CollisionChecker> collision_checker_ptr_;
        std::unique_ptr<Trajectory> previous_trajectory_;

        void timerCallback();


        double curvatureWeight_;
        double decayFactor_;
        double previousVelocity_;
        double timer_callback_dt_;
        double lateral_g_;
        double max_speed_;
        int skip_size_;
        int smooth_size_;
};

#endif