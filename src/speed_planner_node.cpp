#include "speed_planner/speed_planner_node.h"

int main()
{
    //ros::init(argc, argv, "speed_planner");
    SpeedPlannerNode node;
    //ros::spin();

    return 0;
}

SpeedPlannerNode::SpeedPlannerNode() //: nh_(), private_nh_("~"), previous_trajectory_()
{
   
    
    
    /*
    private_nh_.param<double>("mass", mass, 1500.0);
    private_nh_.param<double>("mu", mu, 0.8);
    private_nh_.param<double>("ds", ds, 0.1);
    private_nh_.param<double>("preview_distance", previewDistance, 20.0);
    private_nh_.param<double>("callback_dt", timer_callback_dt_, 0.2);
    private_nh_.param<double>("curvature_weight", curvatureWeight_, 20.0);
    private_nh_.param<double>("decay_factor", decayFactor_, 0.8);
    private_nh_.param<double>("time_weight", weight[0], 0.0);
    private_nh_.param<double>("smooth_weight", weight[1], 15.0);
    private_nh_.param<double>("velocity_weight", weight[2], 0.001);
    private_nh_.param<double>("longitudinal_slack_weight", weight[3], 1.0);
    private_nh_.param<double>("lateral_slack_weight", weight[4], 10.0);
    private_nh_.param<double>("lateral_g", lateral_g_, 0.4);
    private_nh_.param<int>("skip_size", skip_size_, 10);
    private_nh_.param<int>("smooth_size", smooth_size_, 50);
    private_nh_.param<double>("vehicle_length", vehicle_length, 5.0);
    private_nh_.param<double>("vehicle_width", vehicle_width, 1.895);
    private_nh_.param<double>("vehicle_wheel_base", vehicle_wheel_base, 2.790);
    private_nh_.param<double>("vehicle_safety_distance", vehicle_safety_distance, 0.1);
    private_nh_.param<double>("max_speed", max_speed_, 5.0);
    */

    double previewDistance = 20.0;
    double ds = 0.1;
    double mass = 1500.0;
    double mu = 0.8;
    double vehicle_length = 5.0;
    double vehicle_width = 1.895;
    double vehicle_wheel_base = 2.790;
    double vehicle_safety_distance = 0.1;
    std::array<double, 5> weight{0};
    weight[0] = 0.0;
    weight[1] = 15.0;
    weight[2] = 0.001;
    weight[3] = 1.0;
    weight[4] = 10.0;



    speedOptimizer_.reset(new ConvexSpeedOptimizer(previewDistance, ds, mass, mu, weight));
    ego_vehicle_ptr_.reset(new VehicleInfo(vehicle_length, vehicle_width, vehicle_wheel_base,vehicle_safety_distance));
    collision_checker_ptr_.reset(new CollisionChecker());
    

    
}



void SpeedPlannerNode::timerCallback()
{   
  /*
    if(in_lane_ptr_&& in_twist_ptr_ && ego_vehicle_ptr_ && in_pose_ptr_)
    {
        int waypointSize = in_lane_ptr_->waypoints.size();
        std::vector<double> waypoint_x(waypointSize, 0.0);
        std::vector<double> waypoint_y(waypointSize, 0.0);
        std::vector<double> waypoint_yaw(waypointSize, 0.0);
        std::vector<double> waypoint_curvature(waypointSize, 0.0);
        double current_x = in_pose_ptr_->pose.position.x;
        double current_y = in_pose_ptr_->pose.position.y;

        for(size_t id=0; id<waypointSize; ++id)
        {
            waypoint_x[id] = in_lane_ptr_->waypoints[id].pose.pose.position.x;
            waypoint_y[id] = in_lane_ptr_->waypoints[id].pose.pose.position.y;
            waypoint_yaw[id] = tf::getYaw(in_lane_ptr_->waypoints[id].pose.pose.orientation);
            waypoint_curvature[id] = in_lane_ptr_->waypoints[id].pose.pose.position.z;
        }
        
        //1. create trajectory
        int nearest_waypoint_id = getNearestId(current_x, current_y, waypoint_x, waypoint_y);
        Trajectory trajectory(waypoint_x, waypoint_y, waypoint_yaw, waypoint_curvature, nearest_waypoint_id);

        //2. initial speed and initial acceleration
        //initial velocity
        int nearest_previous_point_id=0;
        double v0 = 0.0;
        double a0 = 0.0;

        if(previous_trajectory_==nullptr)
        {
          double v0 = in_twist_ptr_->twist.linear.x;
          previousVelocity_ = v0;
        }
        else
        {
          nearest_previous_point_id = getNearestId(current_x, current_y, previous_trajectory_->x_, previous_trajectory_->y_, 2);
          v0 = previous_trajectory_->velocity_[nearest_previous_point_id];
          a0 = previous_trajectory_->acceleration_[nearest_previous_point_id];
          previousVelocity_ = v0;

          ROS_INFO("Nearest id is %d", nearest_previous_point_id);
        }
        
        ROS_INFO("Value of v0: %f", v0);
        ROS_INFO("Value of a0: %f", a0);
        ROS_INFO("Current Velocity: %f", in_twist_ptr_->twist.linear.x);

        //3. dyanmic obstacles
        std::unique_ptr<CollisionInfo> collision_info_ptr;
        bool is_collide = false;

        if(in_objects_ptr_ && !in_objects_ptr_->objects.empty())
        {
          std::vector<std::shared_ptr<Obstacle>> obstacles;
          obstacles.reserve(in_objects_ptr_->objects.size());

          for(int i=0; i<in_objects_ptr_->objects.size(); ++i)
          {
            if(in_objects_ptr_->objects[i].velocity.linear.x>0.1)
            {
              double obstacle_x = in_objects_ptr_->objects[i].pose.position.x;
              double obstacle_y = in_objects_ptr_->objects[i].pose.position.y;
              double obstacle_angle    = tf::getYaw(in_objects_ptr_->objects[i].pose.orientation);
              double obstacle_radius   = std::sqrt(std::pow(in_objects_ptr_->objects[i].dimensions.x, 2) + std::pow(in_objects_ptr_->objects[i].dimensions.y, 2));
              double obstacle_velocity = in_objects_ptr_->objects[i].velocity.linear.x;

              obstacles.push_back(std::make_shared<DynamicObstacle>(obstacle_x, obstacle_y, obstacle_angle, obstacle_radius, obstacle_velocity, 5, 0.5));
            }
            else
            {
              double obstacle_x = in_objects_ptr_->objects[i].pose.position.x;
              double obstacle_y = in_objects_ptr_->objects[i].pose.position.y;
              double obstacle_angle    = tf::getYaw(in_objects_ptr_->objects[i].pose.orientation);
              double obstacle_radius   = std::sqrt(std::pow(in_objects_ptr_->objects[i].dimensions.x, 2) + std::pow(in_objects_ptr_->objects[i].dimensions.y, 2));

              obstacles.push_back(std::make_shared<StaticObstacle>(obstacle_x, obstacle_y, obstacle_angle, obstacle_radius));
            }
          }

          is_collide = collision_checker_ptr_->check(trajectory, obstacles, ego_vehicle_ptr_, collision_info_ptr);
        }

        if(is_collide)
        {
          assert(collision_info_ptr!=nullptr);
          ROS_INFO("Collide Position id is %d", collision_info_ptr->getId());
          ROS_INFO("Collision Occured Position is %f", trajectory.x_[collision_info_ptr->getId()]);
        }
        else
          ROS_INFO("Not Collide");

        double safeTime = 30.0;

        //4. Create Speed Constraints and Acceleration Constraints
        int N = trajectory.x_.size();
        std::vector<double> Vr(N, 0.0);     //restricted speed array
        std::vector<double> Vd(N, 0.0);     //desired speed array
        std::vector<double> Arlon(N, 0.0);  //acceleration longitudinal restriction
        std::vector<double> Arlat(N, 0.0);  //acceleration lateral restriction
        std::vector<double> Aclon(N, 0.0);  //comfort longitudinal acceleration restriction
        std::vector<double> Aclat(N, 0.0);  //comfort lateral acceleration restriction

        if(is_collide && collision_info_ptr->getType()==Obstacle::TYPE::STATIC)
        {
          Vr[0] = max_speed_;
          Vd[0] = v0;
          for(int i=1; i<collision_info_ptr->getId()-100; ++i)
          {
            Vr[i] = max_speed_;
            Vd[i] = std::max(std::min(Vr[i]-0.5, std::sqrt(lateral_g_/(std::fabs(trajectory.curvature_[i]+1e-10)))), 1.0);
          }
        }
        else if (is_collide && collision_info_ptr->getType()==Obstacle::TYPE::DYNAMIC)
        {
          Vr[0] = v0+0.1;
          Vd[0] = v0;
          for(size_t i=1; i<Vr.size(); ++i)
          {
            Vr[i] = max_speed_;
            Vd[i] = std::max(std::min(Vr[i]-0.5, std::sqrt(lateral_g_/(std::fabs(trajectory.curvature_[i]+1e-10)))), 1.0);
          }
        }
        else
        {
          Vr[0] = v0+0.1;
          Vd[0] = v0;
          for(size_t i=1; i<Vr.size(); ++i)
          {
            Vr[i] = max_speed_;
            Vd[i] = std::max(std::min(Vr[i]-0.5, std::sqrt(lateral_g_/(std::fabs(trajectory.curvature_[i]+1e-10)))), 1.0);
          }
        }

        double mu = speedOptimizer_->mu_;
        for(size_t i=0; i<N; ++i)
        {
            Arlon[i] = 0.5*mu*9.83;
            Arlat[i] = 0.5*mu*9.83;
            Aclon[i] = 0.4*mu*9.83;
            Aclat[i] = 0.4*mu*9.83;
        }


        //Output the information
        ROS_INFO("Size: %d", N);
        if(collision_info_ptr!=nullptr)
        {
          ROS_INFO("Collision id %d", collision_info_ptr->getId());
          ROS_INFO("Collision time %f", collision_info_ptr->getCollisionTime());
        }

        //////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////Calculate Optimized Speed////////////////////////////////
        std::vector<double> result_speed(N, 0.0);
        std::vector<double> result_acceleration(N, 0.0);
        bool is_result = speedOptimizer_->calcOptimizedSpeed(trajectory, 
                                                             result_speed, 
                                                             result_acceleration, 
                                                             Vr, 
                                                             Vd, 
                                                             Arlon, 
                                                             Arlat, 
                                                             Aclon, 
                                                             Aclat, 
                                                             a0, 
                                                             is_collide,
                                                             collision_info_ptr,
                                                             safeTime);

        if(is_result)
        {
          ROS_INFO("Gurobi Success");
          //5. set result
          autoware_msgs::Lane speedOptimizedLane;
          speedOptimizedLane.lane_id = in_lane_ptr_->lane_id;
          speedOptimizedLane.lane_index = in_lane_ptr_->lane_index;
          speedOptimizedLane.is_blocked = in_lane_ptr_->is_blocked;
          speedOptimizedLane.increment = in_lane_ptr_->increment;
          speedOptimizedLane.header = in_lane_ptr_->header;
          speedOptimizedLane.cost = in_lane_ptr_->cost;
          speedOptimizedLane.closest_object_distance = in_lane_ptr_->closest_object_distance;
          speedOptimizedLane.closest_object_velocity = in_lane_ptr_->closest_object_velocity;
          speedOptimizedLane.waypoints.reserve(result_speed.size());

          for(int i=0; i<N; i++)
          {
            result_speed[i] = std::min(result_speed[i] ,max_speed_-0.1);
            autoware_msgs::Waypoint waypoint;
            waypoint.pose.pose.position.x = trajectory.x_[i];
            waypoint.pose.pose.position.y = trajectory.y_[i];
            waypoint.pose.pose.position.z = in_lane_ptr_->waypoints[0].pose.pose.position.z;
            waypoint.pose.pose.orientation = tf::createQuaternionMsgFromYaw(trajectory.yaw_[i]);
            waypoint.pose.header = in_lane_ptr_->header;
            waypoint.twist.header=in_lane_ptr_->header;
            waypoint.twist.twist.linear.x = result_speed[i];

            //std::cout << result_speed[i] << std::endl;

            speedOptimizedLane.waypoints.push_back(waypoint);
          }

          if(!result_speed.empty())
          {
            std_msgs::Float32 result_velocity;
            result_velocity.data = result_speed[0];
            result_velocity_pub_.publish(result_velocity);
            std_msgs::Float32 desired_velocity;
            desired_velocity.data = Vd[2];
            desired_velocity_pub_.publish(desired_velocity);
          }

          optimized_waypoints_pub_.publish(speedOptimizedLane);
          previous_trajectory_.reset(new Trajectory(waypoint_x, waypoint_y, waypoint_yaw, waypoint_curvature, result_speed, result_acceleration));
        }
        else
        {
          ROS_WARN("[Speed Planner]: Gurobi Failed");
          std::cout << "Vr[0]: " << Vr[0] << std::endl;
          std::cout << "Vd[0]: " << Vd[0] << std::endl;
          std::cout << "Vr[1]: " << Vr[1] << std::endl;
          std::cout << "Vd[1]: " << Vd[1] << std::endl;
          std::cout << "Vr[2]: " << Vr[2] << std::endl;
          std::cout << "Vd[2]: " << Vd[2] << std::endl;
          std::cout << "V0: " << v0 << std::endl;
          std::cout << "a0: " << a0 << std::endl;
          //"if gurobi failed calculation"
          if(previous_trajectory_==nullptr)
            return;

          //5. set previous result
          autoware_msgs::Lane speedOptimizedLane;
          speedOptimizedLane.lane_id = in_lane_ptr_->lane_id;
          speedOptimizedLane.lane_index = in_lane_ptr_->lane_index;
          speedOptimizedLane.is_blocked = in_lane_ptr_->is_blocked;
          speedOptimizedLane.increment = in_lane_ptr_->increment;
          speedOptimizedLane.header = in_lane_ptr_->header;
          speedOptimizedLane.cost = in_lane_ptr_->cost;
          speedOptimizedLane.closest_object_distance = in_lane_ptr_->closest_object_distance;
          speedOptimizedLane.closest_object_velocity = in_lane_ptr_->closest_object_velocity;
          speedOptimizedLane.waypoints.reserve(trajectory.x_.size());

          for(int i=nearest_previous_point_id; i<N; i++)
          {
            autoware_msgs::Waypoint waypoint;
            waypoint.pose.pose.position.x = trajectory.x_[i-nearest_previous_point_id];
            waypoint.pose.pose.position.y = trajectory.y_[i-nearest_previous_point_id];
            waypoint.pose.pose.position.z = in_lane_ptr_->waypoints[0].pose.pose.position.z;
            waypoint.pose.pose.orientation = tf::createQuaternionMsgFromYaw(trajectory.yaw_[i-nearest_previous_point_id]);
            waypoint.pose.header = in_lane_ptr_->header;
            waypoint.twist.header=in_lane_ptr_->header;
            waypoint.twist.twist.linear.x = previous_trajectory_->velocity_[i];

            speedOptimizedLane.waypoints.push_back(waypoint);
          }

          optimized_waypoints_pub_.publish(speedOptimizedLane);
        }

        ROS_INFO("=======================================================");
    }
    */

}
