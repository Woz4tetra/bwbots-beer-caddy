#include <bw_description.h>


BwDescription::BwDescription() : Node("bw_description")
{
    _raw_joint_subs = new vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr>();

    this->declare_parameter<string>("joint_frame", "base_link");
    _joints_msg.header.frame_id = this->get_parameter("joint_frame").as_string();

    this->declare_parameter<vector<string>>("joint_names");
    _joint_names = this->get_parameter("joint_names").as_string_array();
    
    this->declare_parameter<double>("update_rate", 15.0);
    _update_rate = this->get_parameter("update_rate").as_double();
    
    for (size_t index = 0; index < _joint_names.size(); index++)
    {
        std::function<void(const std_msgs::msg::Float64::SharedPtr)> callback = 
            std::bind(&BwDescription::joint_callback,
                this, _1, index
        );

        _joints_msg.name.push_back(_joint_names.at(index));
        _joints_msg.position.push_back(0.0);

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_sub = 
            this->create_subscription<std_msgs::msg::Float64>(
                _joints_msg.name.at(index),
                50,
                callback
        );

        _raw_joint_subs->push_back(joint_sub);
    }
    
    _joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 50);

    _loop_timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / _update_rate), std::bind(&BwDescription::loop, this));
    
    RCLCPP_INFO(this->get_logger(), "bw_description is ready!");
}

void BwDescription::joint_callback(std_msgs::msg::Float64::SharedPtr msg, int joint_index)
{
    _joints_msg.position[joint_index] = msg->data;
    _joints_msg.header.stamp = this->get_clock()->now();
}


void BwDescription::loop()
{
    _joint_pub->publish(_joints_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BwDescription>());
    rclcpp::shutdown();
    return 0;
}
