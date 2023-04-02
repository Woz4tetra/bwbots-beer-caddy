#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pluginlib/class_list_macros.h>

/**
 * @brief bw_move_base namespace contains the implementation of MoveAwayRecoveryBehavior
 */
namespace bw_move_base
{
    /**
     * @class MoveAwayRecoveryBehavior
     * @brief A custom RecoveryBehavior implementation that moves the robot away from the nearest obstacle
     * 
     * This class implements a recovery behavior that moves the robot away from the nearest obstacle detected in the global and local costmaps.
     * The behavior calculates the safe distance based on the robot's footprint and publishes velocity commands until the robot reaches a safe distance.
     */
    class MoveAwayRecoveryBehavior : public nav_core::RecoveryBehavior
    {
    public:
        /**
         * @brief Constructor
         */
        MoveAwayRecoveryBehavior();

        /**
         * @brief Initialize the recovery behavior
         * 
         * @param name The name of the recovery behavior
         * @param tf A pointer to the tf2_ros::Buffer object
         * @param global_costmap A pointer to the global costmap
         * @param local_costmap A pointer to the local costmap
         */
        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

        /**
         * @brief Run the recovery behavior
         */
        void runBehavior();

    private:
        double safe_distance_;          //!< The safe distance from obstacles. Computed from footprint
        double safe_distance_buffer_;   //!< The buffer distance added to the safe distance
        double linear_velocity_gain_;   //!< The proportional gain for the linear velocity calculation
        double angular_velocity_gain_;  //!< The proportional gain for the angular velocity calculation
        double frequency_;              //!< The control loop frequency
        double timeout_;
        double safety_waiting_time_;
        geometry_msgs::Twist prev_cmd_vel_;

        double max_linear_acceleration_; //!< The maximum linear acceleration allowed
        double max_angular_acceleration_; //!< The maximum angular acceleration allowed
        double max_linear_velocity_;
        double max_angular_velocity_;

        ros::Publisher cmd_vel_pub_;  //!< Publisher for the velocity commands
        tf2_ros::Buffer* tf_;         //!< Pointer to the tf2_ros::Buffer object
        costmap_2d::Costmap2DROS* global_costmap_; //!< Pointer to the global costmap
        costmap_2d::Costmap2DROS* local_costmap_; //!< Pointer to the local costmap

        std::pair<double, double> findNearestObstacle(costmap_2d::Costmap2D* costmap, const std::vector<costmap_2d::MapLocation>& cells, const geometry_msgs::PoseStamped& robot_pose);
        double getFurthestFootprintDistance(geometry_msgs::Polygon polygon);
        geometry_msgs::Polygon transformPolygonToGlobal(geometry_msgs::Polygon polygon);
        std::vector<costmap_2d::MapLocation> convertPolygonToMapLocations(const geometry_msgs::Polygon& polygon, costmap_2d::Costmap2D* costmap);
        bool transformPolygonToGlobal(const geometry_msgs::Polygon& input_polygon, geometry_msgs::Polygon& output_polygon);
        geometry_msgs::Twist getAccelerationLimitedCommand(double distance, double angle, double dt);

        /**
         * @brief Stop the robot by publishing a zero-velocity command
         */
        void stopRobot();
    };
}
