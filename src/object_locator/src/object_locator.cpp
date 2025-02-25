#include "../include/object_locator.hpp"

namespace object_locator {

    ObjectLocator::ObjectLocator(const rclcpp::NodeOptions &options)
        : Node("object_locator", options) {
        parse_parameters();
        initialize();
    }

    void ObjectLocator::initialize() {
        auto qos = rclcpp::QoS(depth_);
        history_ == "keep_all" ? qos.keep_all() : qos.keep_last(depth_);
        
        pub_ = this->create_publisher<geometry_msgs::msg::Point>("object_position", qos);
        
        auto image_callback = [this](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            
            std::tuple<int, int> location = this->find_object_position(cv_ptr->image);
            geometry_msgs::msg::Point msg_out;
            msg_out.x = std::get<0>(location);
            msg_out.y = std::get<1>(location);
            msg_out.z = 0.0; // Assuming a 2D position, z is set to 0
            
            RCLCPP_INFO(this->get_logger(), "Publishing image_location x = %.2f \t y = %.2f", static_cast<double>(msg_out.x), static_cast<double>(msg_out.y));

            // To see the location on the image it's identifying
            if (msg_out.x >= 0 && msg_out.y >= 0) {
                cv::circle(cv_ptr->image, cv::Point(msg_out.x, msg_out.y), 5, cv::Scalar(0, 255, 0), -1);
            }
            cv::imshow("Detected Object", cv_ptr->image);
            cv::waitKey(1);

            pub_->publish(msg_out);
        };
    
        sub_ = create_subscription<sensor_msgs::msg::Image>("image", qos, image_callback);
    }

    void ObjectLocator::parse_parameters() {
        act_on_ = declare_parameter("act_on", "brightness");
        history_ = declare_parameter("history", "keep_last");
        depth_ = declare_parameter("depth", 1);
        brightness_threshold = declare_parameter("brightness_threshold", 100);
        red_threshold = declare_parameter("red_threshold", 0);
        blue_threshold = declare_parameter("blue_threshold", 0);
        green_threshold = declare_parameter("green_threshold", 250);

        RCLCPP_INFO(this->get_logger(), "brightness threshold: %ld", this->brightness_threshold);
    }

    cv::Mat &ObjectLocator::greyscale_image(cv::Mat &image) {
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
        return image;
    }

    cv::Mat &ObjectLocator::apply_brightness_threshold(cv::Mat &image) {
        // Apply a binary threshold with a fixed threshold value.
        cv::threshold(image, image, brightness_threshold, 255, cv::THRESH_BINARY);
        return image;
    }

    cv::Mat &ObjectLocator::apply_colour_threshold(cv::Mat &image) {
        // Testing with brightness
        return apply_brightness_threshold(image);
    }

    cv::Mat &ObjectLocator::apply_threshold(cv::Mat &image) {
        if (act_on_ == "brightness") {
            return apply_brightness_threshold(image);
        } else if (act_on_ == "colour") {
            return apply_colour_threshold(image);
        } else {
            throw rclcpp::exceptions::InvalidParametersException(
                "Invalid parameter 'act_on_': Value must be one of 'brightness' OR 'colour'");
        }
    }

    std::tuple<int, int> ObjectLocator::find_center_of_gravity(cv::Mat &image) {
        cv::Moments m = cv::moments(image, true);
        if (m.m00 != 0) {
            int cx = static_cast<int>(m.m10 / m.m00);
            int cy = static_cast<int>(m.m01 / m.m00);
            return std::make_tuple(cx, cy);
        }
        // Return an invalid position if no object is found.
        return std::make_tuple(-1, -1);
    }

    std::tuple<int, int> ObjectLocator::find_object_position(cv::Mat &image) {
        cv::Mat processed_image = image;

        // For brightness detection, convert the image to greyscale.
        if (act_on_ == "brightness") {
            greyscale_image(processed_image);
        }
        apply_threshold(processed_image);
        // cv::imshow("Thresholded Image", processed_image);
        // cv::waitKey(1);
        return find_center_of_gravity(processed_image);
    }

} // end of namespace

RCLCPP_COMPONENTS_REGISTER_NODE(object_locator::ObjectLocator)
