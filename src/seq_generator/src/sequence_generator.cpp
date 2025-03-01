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

void SequenceGenerator::initialize() {
    setpoint_left_ = 0;
    setpoint_right_ = 0;
    left_publisher_ = this->create_publisher<example_interfaces::msg::Float64>("/input/left_motor/setpoint_vel", rclcpp::QoS(10));
    right_publisher_ = this->create_publisher<example_interfaces::msg::Float64>("/input/right_motor/setpoint_vel", rclcpp::QoS(10));
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/input/twist", rclcpp::QoS(10));

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
    clock_gettime(CLOCK_MONOTONIC, &test_mode_start_time);
}
void SequenceGenerator::publish_twist_message(double x, double z) {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = x;
    twist_msg.angular.z = z;
    twist_publisher_->publish(twist_msg);
}



void SequenceGenerator::point_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received Point: [x: %f, y: %f]", msg->x, msg->y);

  
  
    if(test_mode_){
        return;
    }
    object_position_x_ = msg->x;
    object_position_y_ = msg->y;
    // msg_out.z = frame_height_ + (frame_width_/1000);
    // frame_height_ = static_cast<int>(msg->z);
    // frame_width_ = static_cast<int>((msg->z - frame_height_)  * 1000);
    frame_width_ = std::modf(msg->z, &frame_height_) *1000;

}


// Callback function for PointStamped messages
void SequenceGenerator::camera_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received Camera position: [x: %f, y: %f]", msg->point.x, msg->point.y);   
    image_center_x_ = msg->point.x;
    image_center_y_ = msg->point.y;  
}


void SequenceGenerator::robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received PoseStamped: [position: (%f, %f), orientation: (%f)], sending setpoints",
    //             msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.z);
    if (test_mode_){
        timespec current_time;
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double elapsed_time = (current_time.tv_sec - test_mode_start_time.tv_sec) +
                    (current_time.tv_nsec - test_mode_start_time.tv_nsec) / 1e9;
        if (elapsed_time < 5) {
            RCLCPP_INFO(this->get_logger(), "0,1");
            publish_twist_message(0, 1);
        } else if (elapsed_time < 10) {
            RCLCPP_INFO(this->get_logger(), "0,-1");
            publish_twist_message(0, -1);
        } else if (elapsed_time < 15) {
            RCLCPP_INFO(this->get_logger(), "1,0");
            publish_twist_message(1, 0);
        }else if (elapsed_time < 20) {
            RCLCPP_INFO(this->get_logger(), "-1,0");
            publish_twist_message(-1, 0);
        }else if (elapsed_time < 25) {
            RCLCPP_INFO(this->get_logger(), "1,1");
            publish_twist_message(1, 1);
        }  else {
            RCLCPP_INFO(this->get_logger(), "0,0");
            publish_twist_message(0, 0);
            test_mode_start_time = current_time;
        }
     }else{
        if (object_position_x_ == -1 && object_position_x_ == -1) {
            publish_twist_message(0, 0);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Frame width: %f", frame_width_);
        double x_error  = (object_position_x_ - (frame_width_ /2));
        double z_error  = x_error > 0 ? -1: 1; 
        double x_distance = abs(object_position_y_ - (frame_height_ /2)) > 0.1 ? 0.5 : 0;
        publish_twist_message(x_distance, z_error);

    }
}

void SequenceGenerator::parse_parameters() {
   test_mode_ = this->declare_parameter("test_mode", true);
}

}  // namespace sequence_generator

RCLCPP_COMPONENTS_REGISTER_NODE(sequence_generator::SequenceGenerator)

