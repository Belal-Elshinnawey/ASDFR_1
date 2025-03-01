#include "../include/sequence_generator.hpp"

namespace sequence_generator {


SequenceGenerator::SequenceGenerator(const rclcpp::NodeOptions &options)
    : Node("sequence_generator", options) {
    parse_parameters();
    initialize();
}

example_interfaces::msg::Float64 SequenceGenerator::create_float64_msg(double value) {
    example_interfaces::msg::Float64 msg;
    msg.data = value;
    return msg;
}
// subscribe to object_position topic
void SequenceGenerator::initialize() {
    setpoint_left_ = 0;
    setpoint_right_ = 0;
    left_publisher_ = this->create_publisher<example_interfaces::msg::Float64>("/input/left_motor/setpoint_vel", rclcpp::QoS(10));
    right_publisher_ = this->create_publisher<example_interfaces::msg::Float64>("/input/right_motor/setpoint_vel", rclcpp::QoS(10));
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/input/twist", rclcpp::QoS(10));

    // auto left_msg = create_float64_msg(setpoint_left_);
    // auto right_msg = create_float64_msg(setpoint_right_);
    // left_publisher_->publish(left_msg);
    // right_publisher_->publish(right_msg);

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    twist_publisher_->publish(twist_msg);
    point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/output/camera_position", 10, std::bind(&SequenceGenerator::camera_position_callback, this, std::placeholders::_1));
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/output/robot_pose", 10, std::bind(&SequenceGenerator::robot_pose_callback, this, std::placeholders::_1));
    object_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
        "object_position", 10, std::bind(&SequenceGenerator::point_callback, this, std::placeholders::_1));
    frame_size_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "frame_size", 10, std::bind(&SequenceGenerator::frame_size_callback, this, std::placeholders::_1));
    
    clock_gettime(CLOCK_MONOTONIC, &test_mode_start_time);

}
void SequenceGenerator::publish_twist_message(double x, double z) {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = x;
    twist_msg.angular.z = z;
    twist_publisher_->publish(twist_msg);
}


void SequenceGenerator::frame_size_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received Frame Size: [width: %f, height: %f]", msg->x, msg->y);
    frame_width_ = msg->x;
    frame_height_ = msg->y;
}

void SequenceGenerator::point_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received Point: [x: %f, y: %f]", msg->x, msg->y);
    if(test_mode_){
        return;
    }
    object_position_x_ = msg->x;
    object_position_y_ = msg->y;

}


// Callback function for PointStamped messages
void SequenceGenerator::camera_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received Camera position: [x: %f, y: %f]", msg->point.x, msg->point.y);   
    image_center_x_ = msg->point.x;
    image_center_y_ = msg->point.y;  
}


void SequenceGenerator::robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received PoseStamped: [position: (%f, %f), orientation: (%f)], sending setpoints",
                msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.z);
    if (test_mode_){
        timespec current_time;
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double elapsed_time = (current_time.tv_sec - test_mode_start_time.tv_sec) +
                    (current_time.tv_nsec - test_mode_start_time.tv_nsec) / 1e9;
        if (elapsed_time < 5) {
            // RCLCPP_INFO(this->get_logger(), "Elapsed time is less than 5 seconds.");
            publish_twist_message(0, 1);
        } else if (elapsed_time < 10) {
            // RCLCPP_INFO(this->get_logger(), "Elapsed time is less than 10 seconds.");
            publish_twist_message(0, -1);
        } else if (elapsed_time < 15) {
            // RCLCPP_INFO(this->get_logger(), "Elapsed time is less than 15 seconds.");
            publish_twist_message(1, 0);
        }else if (elapsed_time < 20) {
            // RCLCPP_INFO(this->get_logger(), "Elapsed time is less than 20 seconds.");
            publish_twist_message(-1, 0);
        }else if (elapsed_time < 25) {
            // RCLCPP_INFO(this->get_logger(), "Elapsed time is less than 25 seconds.");
            publish_twist_message(1, 1);
        }  else {
            // RCLCPP_INFO(this->get_logger(), "Elapsed time is greater than or equal to 20 seconds.");
            publish_twist_message(0, 0);
            test_mode_start_time = current_time;
        }
     }
     else{
        if (object_position_x_ == -1 && object_position_x_ == -1) {
            publish_twist_message(0, 0);
            return;
        }
        
        double tau = 0.001;
        double x_error  = (object_position_x_ - (frame_width_ /2)) * tau;
        double z_error  = 0; 
        z_error = x_error > 0 ? 1: -1;
        if (abs(z_error) > 0.01){
            publish_twist_message(z_error, x_error);
        }else {
            publish_twist_message(0, 0);
        }
    }
}

void SequenceGenerator::parse_parameters() {
   test_mode_ = this->declare_parameter("test_mode", true);
    // in non-test mode, subscribe to the object identifier topic,
    // read the robot pose, if the object on the right, move right
    // if the object on the left, move left
    // if the object is above, move back
    // if the object is below, move forward
    // if the object is in the center, stop.
}

}  // namespace sequence_generator

RCLCPP_COMPONENTS_REGISTER_NODE(sequence_generator::SequenceGenerator)

// image center x pixel
// object position x,y pixel but local


