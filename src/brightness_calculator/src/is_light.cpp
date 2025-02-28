#include "../include/is_light.hpp"

namespace brightness_calculator {

/* Implementation of a very simple node that just logs the bool value of the is_light topic. */

IsLight::IsLight(const rclcpp::NodeOptions &options) : Node("is_light", options) {
    parse_parameters();
    initialize();
}

void IsLight::initialize() {
    RCLCPP_INFO(get_logger(), "Depth #%ld", depth_);
    RCLCPP_INFO(get_logger(), "History #%s", history_.c_str());
    RCLCPP_INFO(get_logger(), "Reliability #%s", reliability_.c_str());
    RCLCPP_INFO(get_logger(), "Durability #%s", durability_.c_str());

    auto qos = rclcpp::QoS(depth_); 
    history_ == "keep_all" ? qos.keep_all() : qos.keep_last(depth_);

    reliability_ == "best_effort" ? qos.best_effort() : qos.reliable();
    durability_ == "transient_local" ? qos.transient_local() : qos.durability_volatile();

    auto log_brightness_callback = [this](std_msgs::msg::Bool::ConstSharedPtr msg) -> void {
        RCLCPP_INFO(get_logger(), "Is Light? #%s", msg ? "True" : "False");
    };

    subscriber = create_subscription<std_msgs::msg::Bool>("is_light", qos, log_brightness_callback);
}

void IsLight::parse_parameters() {
    history_ = this->declare_parameter("history", "keep_last");
    depth_ = this->declare_parameter("depth", 10);
    reliability_ = this->declare_parameter("reliability", "best_effort");
    durability_ = this->declare_parameter("durability", "volatile");
}


} // end of namespace

RCLCPP_COMPONENTS_REGISTER_NODE(brightness_calculator::IsLight);
