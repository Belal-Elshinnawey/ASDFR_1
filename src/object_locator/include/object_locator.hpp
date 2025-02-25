#ifndef OBJECT_LOCATOR_HPP
#define OBJECT_LOCATOR_HPP

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp/exceptions.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <tuple>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.hpp"


namespace object_locator {
    class ObjectLocator : public rclcpp::Node {
        public:
            explicit ObjectLocator(const rclcpp::NodeOptions &options);

        private:
        void parse_parameters();
        void initialize();
        cv::Mat &greyscale_image(cv::Mat &image);
        cv::Mat &apply_brightness_threshold(cv::Mat &image);
        cv::Mat &apply_colour_threshold(cv::Mat &image);
        cv::Mat &apply_threshold(cv::Mat &image);
        std::tuple<int, int> find_center_of_gravity(cv::Mat &image);

        std::tuple<int, int> find_object_position(cv::Mat &image);
        
        std::string act_on_;
        
        size_t brightness_threshold;
        size_t red_threshold;
        size_t blue_threshold;
        size_t green_threshold;

        std::string history_;
        size_t depth_;
        
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;

    };
} 

#endif