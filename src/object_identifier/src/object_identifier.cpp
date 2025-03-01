#include "../include/object_identifier.hpp"

namespace object_identifier {
    ObjectIdentifier::ObjectIdentifier(const rclcpp::NodeOptions &options)
        : Node("object_identifier", options) {
        parse_parameters();
        initialize();
    }

    void ObjectIdentifier::initialize() {
        auto qos = rclcpp::QoS(depth_);
        history_ == "keep_all" ? qos.keep_all() : qos.keep_last(depth_);

        pub_ = this->create_publisher<geometry_msgs::msg::Point>("object_position", qos);
        frame_size_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("frame_size", 10);
        frame_width_ = 0;
        frame_height_ = 0;
        auto image_callback = [this](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            } catch (cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            frame_width_ = cv_ptr->image.cols;
            frame_height_ =  cv_ptr->image.rows;

            std::tuple<int, int> location = this->find_center_of_gravity(cv_ptr->image);
            geometry_msgs::msg::Point msg_out;
            msg_out.x = std::get<0>(location);
            msg_out.y = std::get<1>(location);
            // hay, why not?
            msg_out.z = frame_height_ + (frame_width_/1000);

            RCLCPP_INFO(this->get_logger(), "Publishing image_location x = %.2f \t y = %.2f", static_cast<double>(msg_out.x), static_cast<double>(msg_out.y));

            pub_->publish(msg_out);

            cv::Mat color_image; // to see the dran on circle
            cv::cvtColor(cv_ptr->image, color_image, cv::COLOR_GRAY2BGR);
            cv::circle(color_image, cv::Point(msg_out.x, msg_out.y), 5, cv::Scalar(0, 255, 0), -1);
            cv::imshow("Detected Object on /bw_image", color_image);
            cv::waitKey(1);
        };
        sub_ = create_subscription<sensor_msgs::msg::Image>("bw_image", qos, image_callback);

    }

    void ObjectIdentifier::publish_frame_size() {
        geometry_msgs::msg::Vector3 frame_size_msg;
        frame_size_msg.x = frame_width_;
        frame_size_msg.y = frame_height_;
        frame_size_msg.z = 0.0;
        frame_size_publisher_->publish(frame_size_msg);
    }

    void ObjectIdentifier::parse_parameters() {
        history_ = declare_parameter("history", "keep_last");
        depth_ = declare_parameter("depth", 1);
    }

    std::tuple<int, int> ObjectIdentifier::find_center_of_gravity(cv::Mat &image) {
        cv::Moments m = cv::moments(image, true);
        if (m.m00 != 0) {
            int cx = static_cast<int>(m.m10 / m.m00);
            int cy = static_cast<int>(m.m01 / m.m00);
            return std::make_tuple(cx, cy);
        }
        // Return an invalid position if no object is found.
        return std::make_tuple(-1, -1);
    }

}

RCLCPP_COMPONENTS_REGISTER_NODE(object_identifier::ObjectIdentifier)
