#ifndef BRIGHTNESS_CALCULATOR_HPP
#define BRIGHTNESS_CALCULATOR_HPP

#include <boost/asio.hpp>
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <ctime>
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"



namespace sequence_generator {
class SequenceGenerator : public rclcpp::Node {
 public:
  explicit SequenceGenerator(const rclcpp::NodeOptions &options);

 private:
    double setpoint_left_;
    double setpoint_right_;
    bool test_mode_;
    double object_position_x_;
    double object_position_y_;
    double image_center_x_;
    double image_center_y_;  
    double frame_width_;
    double frame_height_;
    timespec test_mode_start_time;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_publisher_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;  

    void initialize();
    void parse_parameters();
    example_interfaces::msg::Float64 create_float64_msg(double value);
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr object_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr frame_size_subscriber_;

    void camera_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void point_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void frame_size_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void publish_twist_message(double x, double z);

};

}  // namespace brightness_calculator

#endif


