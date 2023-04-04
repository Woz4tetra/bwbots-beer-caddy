#include "move_away_recovery_behavior.h"

PLUGINLIB_EXPORT_CLASS(bw_move_base::MoveAwayRecoveryBehavior, nav_core::RecoveryBehavior)


using namespace bw_move_base;


MoveAwayRecoveryBehavior::MoveAwayRecoveryBehavior() : global_costmap_(NULL)
{
}

void MoveAwayRecoveryBehavior::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
    // Check if global and local costmaps are initialized
    if (!global_costmap || !local_costmap)
    {
        // If either costmap is not initialized, log an error and return
        ROS_ERROR("Global and local costmaps must be initialized.");
        return;
    }

    // Set the tf, global_costmap_, and local_costmap_ class members
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    // Create a private node handle with the given name
    ros::NodeHandle private_nh("~/" + name);

    // Load parameters from the parameter server
    private_nh.param("safe_distance_buffer", safe_distance_buffer_, 0.1);
    private_nh.param("linear_velocity_gain", linear_velocity_gain_, 0.1);
    private_nh.param("angular_velocity_gain", angular_velocity_gain_, 0.5);
    private_nh.param("frequency", frequency_, 10.0);
    private_nh.param("max_linear_acceleration", max_linear_acceleration_, 0.1);
    private_nh.param("max_angular_acceleration", max_angular_acceleration_, 0.5);
    private_nh.param("max_linear_velocity", max_linear_velocity_, 1.0);
    private_nh.param("max_angular_velocity", max_angular_velocity_, 1.57);
    private_nh.param("timeout", timeout_, 10.0);
    private_nh.param("safety_waiting_time", safety_waiting_time_, 1.0);
    private_nh.param("inflation_radius_", inflation_radius_, 1.0);

    // Create a node handle for general use
    ros::NodeHandle nh;

    // Advertise the cmd_vel topic to publish velocity commands
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}


double MoveAwayRecoveryBehavior::getFurthestFootprintDistance(geometry_msgs::Polygon polygon)
{
    // Initialize the maximum distance variable to 0.0
    double max_distance = 0.0;

    // Iterate through all the points in the input polygon
    for (const geometry_msgs::Point32& point : polygon.points)
    {
        // Calculate the distance between the robot's reference point and the current point
        double distance = std::hypot(point.x, point.y);

        // Update the maximum distance if the current distance is greater
        max_distance = std::max(max_distance, distance);
    }

    // Return the maximum distance found
    return max_distance;
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
    ros::Time start_time = ros::Time::now();
    ros::Time safe_time = ros::Time::now();

    safe_distance_ = getFurthestFootprintDistance(global_costmap_->getRobotFootprintPolygon());
    safe_distance_ += safe_distance_buffer_;

    // Get the global costmap and its data
    costmap_2d::Costmap2D* global_costmap = global_costmap_->getCostmap();

    geometry_msgs::Polygon robot_footprint = global_costmap_->getRobotFootprintPolygon();
    geometry_msgs::Polygon inflated_robot_footprint = inflatePolygon(robot_footprint, inflation_radius_);
    ROS_INFO_STREAM("inflated_robot_footprint: " << inflated_robot_footprint);

    while (ros::ok() && ros::Time::now() - start_time < ros::Duration(timeout_))
    {
        geometry_msgs::Polygon global_footprint;
        if (!transformPolygonToGlobal(inflated_robot_footprint, global_footprint)) {
            continue;
        }

        // Get the robot's pose in the global frame
        geometry_msgs::PoseStamped robot_pose;
        if (!global_costmap_->getRobotPose(robot_pose))
        {
            ROS_ERROR("Failed to get robot pose");
            return;
        }

        // Get the cells within the global footprint polygon
        std::vector<costmap_2d::MapLocation> footprintMapLocations = convertPolygonToMapLocations(global_footprint, global_costmap);
        std::vector<costmap_2d::MapLocation> cells;
        global_costmap->polygonOutlineCells(footprintMapLocations, cells);        

        // Find the distance and angle to the nearest obstacle
        std::pair<double, double> nearest_obstacle = findNearestObstacle(global_costmap, cells, robot_pose);
        double distance = nearest_obstacle.first;
        double angle = nearest_obstacle.second;
        ROS_INFO("distance: %f, angle: %f", distance, angle);

        if (distance >= safe_distance_)
        {
            ROS_INFO("The robot is at a safe distance (%f > %f).", distance, safe_distance_);
            
            if (ros::Time::now() - safe_time > ros::Duration(safety_waiting_time_))
            {
                ROS_INFO("The robot has been at a safe distance for more than %f second. Exiting.", safety_waiting_time_);
                break;
            }
        }
        else
        {
            safe_time = ros::Time::now();
        }

        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;
        
        cmd_vel_pub_.publish(getAccelerationLimitedCommand(distance, angle, dt));

        rate.sleep();
    }

    stopRobot();
}

geometry_msgs::Twist MoveAwayRecoveryBehavior::getAccelerationLimitedCommand(double distance, double angle, double dt)
{
    if (abs(angle) > M_PI / 2.0) {
        distance *= -1.0;
        angle *= -1.0;
    }
    // Calculate linear and angular velocities based on the distance and angle to the nearest obstacle
    double linear_velocity = -linear_velocity_gain_ * distance;
    double angular_velocity = angular_velocity_gain_ * angle;

    // Limit acceleration
    double linear_diff = linear_velocity - prev_cmd_vel_.linear.x;
    double angular_diff = angular_velocity - prev_cmd_vel_.angular.z;

    double max_linear_diff = max_linear_acceleration_ * dt;
    double max_angular_diff = max_angular_acceleration_ * dt;

    // Clamp the difference in linear and angular velocities to the maximum allowed acceleration
    linear_diff = std::max(std::min(linear_diff, max_linear_diff), -max_linear_diff);
    angular_diff = std::max(std::min(angular_diff, max_angular_diff), -max_angular_diff);

    // Calculate the new limited linear and angular velocities
    double limited_linear_velocity = prev_cmd_vel_.linear.x + linear_diff;
    double limited_angular_velocity = prev_cmd_vel_.angular.z + angular_diff;

    // Clamp the limited linear and angular velocities to the maximum allowed velocities
    limited_linear_velocity = std::max(std::min(limited_linear_velocity, max_linear_velocity_), -max_linear_velocity_);
    limited_angular_velocity = std::max(std::min(limited_angular_velocity, max_angular_velocity_), -max_angular_velocity_);

    // Create and populate the Twist message with the limited linear and angular velocities
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = limited_linear_velocity;
    cmd_vel.angular.z = limited_angular_velocity;
    prev_cmd_vel_ = cmd_vel;

    // Return the acceleration-limited velocity command
    return cmd_vel;
}


bool MoveAwayRecoveryBehavior::transformPolygonToGlobal(const geometry_msgs::Polygon& input_polygon, geometry_msgs::Polygon& output_polygon)
{
    // Get the global frame and robot base frame
    std::string global_frame = global_costmap_->getGlobalFrameID();
    std::string robot_frame = global_costmap_->getBaseFrameID();

    // Look up the transform from robot_frame to global_frame
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_->lookupTransform(global_frame, robot_frame, ros::Time(0), ros::Duration(0.5));
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "Failed to transform the polygon: %s", ex.what());
        return false;
    }

    // Iterate through the points in the input_polygon
    for (const geometry_msgs::Point32& input_point : input_polygon.points)
    {
        geometry_msgs::PointStamped input_point_stamped;
        geometry_msgs::PointStamped output_point_stamped;

        // Set the input point to a stamped version
        input_point_stamped.point.x = input_point.x;
        input_point_stamped.point.y = input_point.y;
        input_point_stamped.point.z = input_point.z;

        // Set the header frame_id and stamp
        input_point_stamped.header.frame_id = robot_frame;
        input_point_stamped.header.stamp = ros::Time::now();

        // Transform the point to the global frame using tf2::doTransform
        tf2::doTransform(input_point_stamped, output_point_stamped, transform);

        // Add the transformed point to the output polygon
        geometry_msgs::Point32 transformed_point;
        transformed_point.x = output_point_stamped.point.x;
        transformed_point.y = output_point_stamped.point.y;
        transformed_point.z = output_point_stamped.point.z;

        output_polygon.points.push_back(transformed_point);
    }

    return true;
}

geometry_msgs::Polygon MoveAwayRecoveryBehavior::inflatePolygon(const geometry_msgs::Polygon& polygon, double inflation_radius)
{
    geometry_msgs::Polygon inflated_polygon;

    // Iterate through the points in the polygon
    for (size_t i = 0; i < polygon.points.size(); ++i)
    {
        // Get the previous, current, and next points in the polygon
        const geometry_msgs::Point32& prev_point = polygon.points[(i + polygon.points.size() - 1) % polygon.points.size()];
        const geometry_msgs::Point32& current_point = polygon.points[i];
        const geometry_msgs::Point32& next_point = polygon.points[(i + 1) % polygon.points.size()];

        // Calculate the vectors between the points
        geometry_msgs::Point32 prev_vec, next_vec;
        prev_vec.x = current_point.x - prev_point.x;
        prev_vec.y = current_point.y - prev_point.y;
        next_vec.x = next_point.x - current_point.x;
        next_vec.y = next_point.y - current_point.y;

        // Normalize the vectors
        double prev_vec_length = std::hypot(prev_vec.x, prev_vec.y);
        double next_vec_length = std::hypot(next_vec.x, next_vec.y);
        prev_vec.x /= prev_vec_length;
        prev_vec.y /= prev_vec_length;
        next_vec.x /= next_vec_length;
        next_vec.y /= next_vec_length;

        // Calculate the average of the two normalized vectors
        geometry_msgs::Point32 avg_vec;
        avg_vec.x = (prev_vec.x + next_vec.x) / 2.0;
        avg_vec.y = (prev_vec.y + next_vec.y) / 2.0;

        // Normalize the average vector and scale by the inflation radius
        double avg_vec_length = std::hypot(avg_vec.x, avg_vec.y);
        avg_vec.x = (avg_vec.x / avg_vec_length) * inflation_radius;
        avg_vec.y = (avg_vec.y / avg_vec_length) * inflation_radius;

        // Calculate the inflated point
        geometry_msgs::Point32 inflated_point;
        inflated_point.x = current_point.x + avg_vec.x;
        inflated_point.y = current_point.y + avg_vec.y;

        // Add the inflated point to the inflated_polygon
        inflated_polygon.points.push_back(inflated_point);
    }

    return inflated_polygon;
}

std::vector<costmap_2d::MapLocation> MoveAwayRecoveryBehavior::convertPolygonToMapLocations(const geometry_msgs::Polygon& polygon, costmap_2d::Costmap2D* costmap)
{
    std::vector<costmap_2d::MapLocation> mapLocations;

    // Iterate through the points in the polygon
    for (const auto& point : polygon.points)
    {
        // Convert the point's world coordinates to map coordinates in the costmap
        costmap_2d::MapLocation map_location;
        costmap->worldToMap(point.x, point.y, map_location.x, map_location.y);

        // Add the converted map location to the vector of map locations
        mapLocations.push_back(map_location);
    }

    // Return the vector of map locations corresponding to the polygon's points
    return mapLocations;
}

void MoveAwayRecoveryBehavior::stopRobot()
{
    ROS_INFO("Setting robot velocity to zero.");
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    cmd_vel_pub_.publish(cmd_vel);
}

std::pair<double, double> MoveAwayRecoveryBehavior::findNearestObstacle(costmap_2d::Costmap2D* costmap, const std::vector<costmap_2d::MapLocation>& cells, const geometry_msgs::PoseStamped& robot_pose)
{
    double min_distance = safe_distance_;
    double sum_angle = 0.0;
    double avg_angle = 0.0;
    int count = 0;

    // Iterate through the cells in the costmap
    for (const auto& cell : cells)
    {
        // Skip cells that are not lethal obstacles
        if (costmap->getCost(cell.x, cell.y) != costmap_2d::LETHAL_OBSTACLE) {
            continue;
        }

        // Convert cell coordinates to world coordinates
        double wx, wy;
        costmap->mapToWorld(cell.x, cell.y, wx, wy);

        // Calculate the distance and angle from the robot's pose to the obstacle
        double distance = std::hypot(wx - robot_pose.pose.position.x, wy - robot_pose.pose.position.y);
        double angle = std::atan2(wy - robot_pose.pose.position.y, wx - robot_pose.pose.position.x) + M_PI;

        // Accumulate the distance and angle to compute average later
        if (distance < min_distance) {
            min_distance = distance;
        }
        sum_angle += angle;
        count++;
    }

    // Compute the average distance and angle to the nearest obstacle
    if (count == 0) {
        avg_angle = 0.0;
    }
    else {
        avg_angle = sum_angle / count;
        avg_angle -= M_PI;
    }

    // Return the average distance and angle to the nearest obstacle
    return {min_distance, avg_angle};
}