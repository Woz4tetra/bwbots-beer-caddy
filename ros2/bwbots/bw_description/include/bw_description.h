#pragma once

#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


using namespace std;
using namespace std::chrono_literals;
using namespace std::placeholders;

class BwDescription : public rclcpp::Node {
private:
    // Parameters
    std::vector<std::string> _joint_names;
    double _update_rate;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_pub;

    // Subscribers
    vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr>* _raw_joint_subs;

    // Members
    rclcpp::TimerBase::SharedPtr _loop_timer;

    // Messages
    sensor_msgs::msg::JointState _joints_msg;

    // Module callback
    void joint_callback(std_msgs::msg::Float64::SharedPtr msg, int joint_index);

    // Main loop methods
    void loop();
public:
    BwDescription();
};
