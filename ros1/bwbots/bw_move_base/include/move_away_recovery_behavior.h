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
 * @brief bw_move_base namespace contains the implementation of the MoveAwayRecoveryBehavior class
 */
namespace bw_move_base
{
    /**
     * @class MoveAwayRecoveryBehavior
     * @brief A custom RecoveryBehavior implementation that moves the robot away from the nearest obstacle
     * 
     * This class implements a recovery behavior that moves the robot away from the nearest obstacle detected in the global costmap.
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
         * 
         * This method contains the main control loop for the recovery behavior. It publishes velocity commands
         * to move the robot away from obstacles until it reaches a safe distance or the timeout is reached.
         */
        void runBehavior();

    private:
        double safe_distance_;          //!< The safe distance from obstacles, computed from the robot's footprint
        double safe_distance_buffer_;   //!< Additional buffer distance added to the safe distance
        double linear_velocity_gain_;   //!< Proportional gain for the linear velocity calculation
        double angular_velocity_gain_;  //!< Proportional gain for the angular velocity calculation
        double frequency_;              //!< Control loop frequency (Hz)
        double timeout_;                //!< Maximum duration for the recovery behavior to run (seconds)
        double safety_waiting_time_;    //!< Duration to wait after robot is safe before exiting the behavior (seconds)
        geometry_msgs::Twist prev_cmd_vel_; //!< Previous command velocity, used for acceleration limiting

        double max_linear_acceleration_; //!< Maximum allowed linear acceleration (m/s^2)
        double max_angular_acceleration_; //!< Maximum allowed angular acceleration (rad/s^2)
        double max_linear_velocity_;    //!< Maximum allowed linear velocity (m/s)
        double max_angular_velocity_;   //!< Maximum allowed angular velocity (rad/s)
        double inflation_radius_;       //!< How much to inflate the robot footprint by for safety checks (m)

        ros::Publisher cmd_vel_pub_;    //!< Publisher for the velocity commands
        tf2_ros::Buffer* tf_;           //!< Pointer to the tf2_ros::Buffer object
        costmap_2d::Costmap2DROS* global_costmap_; //!< Pointer to the global costmap
        costmap_2d::Costmap2DROS* local_costmap_; //!< Pointer to the local costmap

        /**
         * @brief Finds the distance and angle to the nearest obstacle within the specified cells
         *
         * @param costmap Pointer to the costmap_2d::Costmap2D object representing the global costmap
         * @param cells Vector of costmap_2d::MapLocation objects representing the cells within the footprint polygon
         * @param robot_pose geometry_msgs::PoseStamped object representing the robot's pose in the global frame
         * @return A pair containing the distance (first) and angle (second) to the nearest obstacle
         */
        std::pair<double, double> findNearestObstacle(costmap_2d::Costmap2D* costmap, const std::vector<costmap_2d::MapLocation>& cells, const geometry_msgs::PoseStamped& robot_pose);

        /**
         * @brief Computes the furthest distance between any two points in the given polygon
         *
         * @param polygon geometry_msgs::Polygon object representing the footprint polygon
         * @return The maximum distance between any two points in the polygon
         */
        double getFurthestFootprintDistance(geometry_msgs::Polygon polygon);

        /**
         * @brief Transforms a given polygon using the class member tf_
         *
         * @param polygon geometry_msgs::Polygon object representing the input polygon
         * @return Transformed geometry_msgs::Polygon object in the global frame
         */
        geometry_msgs::Polygon transformPolygonToGlobal(geometry_msgs::Polygon polygon);

        /**
         * @brief Converts a polygon to a vector of costmap_2d::MapLocation objects
         *
         * @param polygon geometry_msgs::Polygon object representing the input polygon
         * @param costmap Pointer to the costmap_2d::Costmap2D object representing the global costmap
         * @return Vector of costmap_2d::MapLocation objects representing the cells within the polygon
         */
        std::vector<costmap_2d::MapLocation> convertPolygonToMapLocations(const geometry_msgs::Polygon& polygon, costmap_2d::Costmap2D* costmap);

        /**
         * @brief Inflate a given polygon by a specified radius.
         * This function creates a new polygon by inflating the input polygon by the given inflation radius. It calculates
         * the normal at each vertex of the input polygon and moves the vertices outward by the specified inflation radius.
         * The newly calculated vertices form the inflated polygon.
         * 
         * @param polygon The input polygon to be inflated.
         * @param inflation_radius The radius by which to inflate the polygon.
         * @return geometry_msgs::Polygon The inflated polygon.
         */
        geometry_msgs::Polygon inflatePolygon(const geometry_msgs::Polygon& polygon, double inflation_radius);

        /**
         * @brief Transforms a given input polygon to the global frame
         *
         * @param input_polygon geometry_msgs::Polygon object representing the input polygon in the robot's frame
         * @param output_polygon Reference to a geometry_msgs::Polygon object where the transformed polygon will be stored
         * @return True if the transformation is successful, false otherwise
         */
        bool transformPolygonToGlobal(const geometry_msgs::Polygon& input_polygon, geometry_msgs::Polygon& output_polygon);

        /**
         * @brief Generates an acceleration-limited command based on the distance and angle to the nearest obstacle
         *
         * @param distance The distance to the nearest obstacle
         * @param angle The angle to the nearest obstacle
         * @param dt The time step since the last update
         * @return geometry_msgs::Twist object containing the acceleration-limited linear and angular velocity commands
         */
        geometry_msgs::Twist getAccelerationLimitedCommand(double distance, double angle, double dt);

        /**
         * @brief Stop the robot by publishing a zero-velocity command
         * 
         * This method publishes a zero-velocity command to stop the robot's movement.
         */
        void stopRobot();
    };
}
