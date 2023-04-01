#include "move_away_recovery_behavior.h"

PLUGINLIB_EXPORT_CLASS(move_away_recovery_behavior::MoveAwayRecoveryBehavior, nav_core::RecoveryBehavior)


using namespace move_away_recovery_behavior;


MoveAwayRecoveryBehavior::MoveAwayRecoveryBehavior() : global_costmap_(NULL)
{
}

void MoveAwayRecoveryBehavior::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("safe_distance", safe_distance_, 1.0);
    private_nh.param("linear_velocity", linear_velocity_, 0.1);
    private_nh.param("angular_velocity_gain", angular_velocity_gain_, 0.5);
    private_nh.param("frequency", frequency_, 10.0);
    private_nh.param("max_linear_acceleration", max_linear_acceleration_, 0.1);
    private_nh.param("max_angular_acceleration", max_angular_acceleration_, 0.5);
    private_nh.param("time_step", time_step_, 0.1);

    ros::NodeHandle nh;
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void MoveAwayRecoveryBehavior::runBehavior()
{
    if (!global_costmap_)
    {
        ROS_ERROR("Global costmap is not initialized.");
        return;
    }

    ros::Rate rate(frequency_);
    bool safe = false;
    geometry_msgs::Twist prev_cmd_vel;
    
    double max_linear_diff = max_linear_acceleration_ * time_step_;
    double max_angular_diff = max_angular_acceleration_ * time_step_;

    while (ros::ok() && !safe)
    {
        std::pair<double, double> nearest_obstacle = findNearestObstacle();
        double distance = nearest_obstacle.first;
        double angle = nearest_obstacle.second;

        if (distance > safe_distance_)
        {
            safe = true;
        }
        else
        {
            // Calculate linear and angular velocities based on the distance and angle to the nearest obstacle
            double linear_velocity = -linear_velocity_ * distance;
            double angular_velocity = -angular_velocity_gain_ * angle;

            // Limit acceleration
            
            double linear_diff = linear_velocity - prev_cmd_vel.linear.x;
            double angular_diff = angular_velocity - prev_cmd_vel.angular.z;

            linear_diff = std::max(std::min(linear_diff, max_linear_diff), -max_linear_diff);
            angular_diff = std::max(std::min(angular_diff, max_angular_diff), -max_angular_diff);

            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = prev_cmd_vel.linear.x + linear_diff;
            cmd_vel.angular.z = prev_cmd_vel.angular.z + angular_diff;

            cmd_vel_pub_.publish(cmd_vel);
            prev_cmd_vel = cmd_vel;
        }

        rate.sleep();
    }

    stopRobot();
}

void MoveAwayRecoveryBehavior::stopRobot()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    cmd_vel_pub_.publish(cmd_vel);
}

std::pair<double, double> MoveAwayRecoveryBehavior::findNearestObstacle()
{
    double min_distance = std::numeric_limits<double>::max();
    double min_angle = 0.0;

    std::vector<costmap_2d::Costmap2DROS*> costmaps = {global_costmap_, local_costmap_};

    for (costmap_2d::Costmap2DROS* costmap_ros : costmaps)
    {
        costmap_2d::Costmap2D* costmap = costmap_ros->getCostmap();
        unsigned int map_size = costmap->getSizeInCellsX() * costmap->getSizeInCellsY();
        unsigned int mx, my;
        double wx, wy;

        std::string global_frame = costmap_ros->getGlobalFrameID();
        std::string robot_frame = costmap_ros->getBaseFrameID();

        for (unsigned int i = 0; i < map_size; ++i)
        {
            costmap->indexToCells(i, mx, my);
            if (costmap->getCost(mx, my) == costmap_2d::LETHAL_OBSTACLE)
            {
                costmap->mapToWorld(mx, my, wx, wy);

                // Transform the obstacle coordinates to the robot's local frame
                geometry_msgs::PointStamped obstacle_global, obstacle_robot;
                obstacle_global.header.stamp = ros::Time(0);
                obstacle_global.header.frame_id = global_frame;
                obstacle_global.point.x = wx;
                obstacle_global.point.y = wy;
                obstacle_global.point.z = 0;

                try
                {
                    tf_->transform(obstacle_global, obstacle_robot, robot_frame);
                }
                catch (tf2::TransformException& ex)
                {
                    ROS_WARN("Could not transform obstacle coordinates: %s", ex.what());
                    continue;
                }

                double dx = obstacle_robot.point.x;
                double dy = obstacle_robot.point.y;
                double distance = std::hypot(dx, dy);
                double angle = std::atan2(dy, dx);

                if (distance < min_distance)
                {
                    min_distance = distance;
                    min_angle = angle;
                }
            }
        }
    }

    return std::make_pair(min_distance, min_angle);
}
