#include "move_away_recovery_behavior.h"

PLUGINLIB_EXPORT_CLASS(bw_move_base::MoveAwayRecoveryBehavior, nav_core::RecoveryBehavior)


using namespace bw_move_base;


MoveAwayRecoveryBehavior::MoveAwayRecoveryBehavior() : global_costmap_(NULL)
{
}

void MoveAwayRecoveryBehavior::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
    if (!global_costmap || !local_costmap)
    {
        ROS_ERROR("Global and local costmaps must be initialized.");
        return;
    }

    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("safe_distance_buffer", safe_distance_buffer_, 0.1);
    private_nh.param("linear_velocity_gain", linear_velocity_gain_, 0.1);
    private_nh.param("angular_velocity_gain", angular_velocity_gain_, 0.5);
    private_nh.param("frequency", frequency_, 10.0);
    private_nh.param("max_linear_acceleration", max_linear_acceleration_, 0.1);
    private_nh.param("max_angular_acceleration", max_angular_acceleration_, 0.5);

    ros::NodeHandle nh;
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    footprint_sub_ = nh.subscribe("local_costmap/footprint", 1, &MoveAwayRecoveryBehavior::footprintCallback, this);
}

void MoveAwayRecoveryBehavior::footprintCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
    double max_distance = 0.0;

    for (const geometry_msgs::Point32& point : msg->polygon.points)
    {
        double distance = std::hypot(point.x, point.y);
        max_distance = std::max(max_distance, distance);
    }

    safe_distance_ = max_distance + safe_distance_buffer_;
}

void MoveAwayRecoveryBehavior::runBehavior()
{
    ROS_WARN("Running move away recovery behavior");
    if (!global_costmap_)
    {
        ROS_ERROR("Global costmap is not initialized.");
        return;
    }

    ros::Rate rate(frequency_);
    ros::Time last_time = ros::Time::now(); // Store the time of the last tick
    geometry_msgs::Twist prev_cmd_vel;
    
    while (ros::ok())
    {
        std::pair<double, double> nearest_obstacle = findNearestObstacle();

        double distance = nearest_obstacle.first;
        double angle = nearest_obstacle.second;

        if (distance > safe_distance_)
        {
            ROS_INFO("The robot is at a safe distance. Exiting.");
            break;
        }

        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;
        
        // Calculate linear and angular velocities based on the distance and angle to the nearest obstacle
        double linear_velocity = -linear_velocity_gain_ * distance;
        double angular_velocity = -angular_velocity_gain_ * angle;

        // Limit acceleration
        
        double linear_diff = linear_velocity - prev_cmd_vel.linear.x;
        double angular_diff = angular_velocity - prev_cmd_vel.angular.z;

        double max_linear_diff = max_linear_acceleration_ * dt;
        double max_angular_diff = max_angular_acceleration_ * dt;

        linear_diff = std::max(std::min(linear_diff, max_linear_diff), -max_linear_diff);
        angular_diff = std::max(std::min(angular_diff, max_angular_diff), -max_angular_diff);

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = prev_cmd_vel.linear.x + linear_diff;
        cmd_vel.angular.z = prev_cmd_vel.angular.z + angular_diff;

        cmd_vel_pub_.publish(cmd_vel);
        prev_cmd_vel = cmd_vel;

        rate.sleep();
    }

    stopRobot();
}

void MoveAwayRecoveryBehavior::stopRobot()
{
    ROS_INFO("Setting robot velocity to zero.");
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

    std::string global_frame = global_costmap_->getGlobalFrameID();
    std::string robot_frame = global_costmap_->getBaseFrameID();

    geometry_msgs::TransformStamped global_to_robot_transform;
    try
    {
        global_to_robot_transform = tf_->lookupTransform(
            robot_frame, global_frame,
            ros::Time(0), ros::Duration(1.0)
        );
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "Could not transform obstacle coordinates: %s", ex.what());
        return std::make_pair(0.0, 0.0);
    }

    for (costmap_2d::Costmap2DROS* costmap_ros : costmaps)
    {
        costmap_2d::Costmap2D* costmap = costmap_ros->getCostmap();
        unsigned int map_size = costmap->getSizeInCellsX() * costmap->getSizeInCellsY();
        unsigned int mx, my;
        double wx, wy;

        for (unsigned int i = 0; i < map_size; ++i)
        {
            costmap->indexToCells(i, mx, my);
            if (costmap->getCost(mx, my) == costmap_2d::LETHAL_OBSTACLE)
            {
                costmap->mapToWorld(mx, my, wx, wy);

                // Transform the obstacle coordinates to the robot's local frame
                geometry_msgs::PoseStamped obstacle_global, obstacle_robot;
                obstacle_global.header.stamp = ros::Time(0);
                obstacle_global.header.frame_id = global_frame;
                obstacle_global.pose.position.x = wx;
                obstacle_global.pose.position.y = wy;
                obstacle_global.pose.position.z = 0;
                obstacle_global.pose.orientation.w = 1.0;

                tf2::doTransform(obstacle_global, obstacle_robot, global_to_robot_transform);

                double dx = obstacle_robot.pose.position.x;
                double dy = obstacle_robot.pose.position.y;
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
