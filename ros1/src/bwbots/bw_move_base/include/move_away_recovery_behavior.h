#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>


namespace move_away_recovery_behavior
{

    class MoveAwayRecoveryBehavior : public nav_core::RecoveryBehavior
    {
    public:
        MoveAwayRecoveryBehavior();
        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);
        void runBehavior();

    private:
        double safe_distance_;
        double linear_velocity_;
        double angular_velocity_gain_;
        double frequency_;

        double max_linear_acceleration_;
        double max_angular_acceleration_;
        double time_step_;

        ros::Publisher cmd_vel_pub_;
        tf2_ros::Buffer* tf_;
        costmap_2d::Costmap2DROS* global_costmap_;
        costmap_2d::Costmap2DROS* local_costmap_;
        std::pair<double, double> findNearestObstacle();
        void stopRobot();
    };
}
